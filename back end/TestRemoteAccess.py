"""Who may use the bridge (RemoteAccess.py; 2026-10-07 audit 2 H2 and M2, audit 1 S5; Aaran's decision 2), on the fake
motor bus through Starlette's test client: no serial port, no camera, no live bridge, no network. The password and
connect-intent files live in a scratch folder (SO101_REMOTE_PASSWORD_FILE, SO101_CONNECT_INTENT_FILE).

Rules   Origin, loopback and Host checks; DNS rebinding and local proxies are not this PC.
Local   this PC works exactly as before: no password, state first on /ws, commands, programs without an Origin.
Site    a page from any other site (or the bridge's own /docs, or another local web app's port) is refused outright.
Remote  another device gets only /health without the password, everything with it; /ws asks for it first; a wrong
        one is refused; five different wrong ones lock that address out, the same one again does not; CORS lets the
        page read the refusal.
Intent  Disconnect and a page's Emergency Shutoff keep "disconnected on purpose" (on disk too) until an explicit Connect;
        another device never connects by itself.
Setter  SetRemotePassword.py stores a hash, never the password; a new password works without a restart.

Run:  python TestRemoteAccess.py
"""
import asyncio
import io
import json
import os
import shutil
import tempfile
import threading
import time
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("SO101_DEPTH_CAMERA", "0")
os.environ.setdefault("SO101_RECORDER", "0")
Scratch = Path(tempfile.mkdtemp(prefix="TestRemoteAccess"))
os.environ["SO101_REMOTE_PASSWORD_FILE"] = str(Scratch / "RemotePassword.json")
os.environ["SO101_CONNECT_INTENT_FILE"] = str(Scratch / "ConnectIntent.json")

from starlette.websockets import WebSocketDisconnect  # noqa: E402

import bridge_server as Bridge  # noqa: E402
import RemoteAccess  # noqa: E402
import SetRemotePassword  # noqa: E402
from TestSafety import ClientAt, LocalClient, MakeController  # noqa: E402
from test_bridge_home import FakeBus  # noqa: E402

Password = "correct horse battery"
PcLan = "192.168.1.20"            # the robot's PC as another device names it
Phone = ("192.168.1.30", 50123)   # another device on the LAN
PhoneOrigin = f"http://{PcLan}:5181"


def Remote(Client=Phone, Host=PcLan):
    """The test client as another device (Client) asking for the bridge at Host:8000."""
    return ClientAt(Bridge.app, base_url=f"http://{Host}:8000", client=Client)


def Headers(Origin=PhoneOrigin, Key=None, **Extra):
    Result = {"origin": Origin} if Origin else {}
    if Key is not None:
        Result[RemoteAccess.PasswordHeader] = Key
    return {**Result, **Extra}


class ConnectingBus(FakeBus):
    """The fake motor bus as connect meets the arm: it opens, and the Goal_Position seeds energise every servo."""

    def connect(Self):
        Self.is_connected = True

    def configure_motors(Self, acceleration: int = 0, goal_speed: int = 0) -> None:
        super().configure_motors(acceleration, goal_speed)
        Self.torque_enabled = True


class Base(unittest.TestCase):
    def setUp(Self):
        RemoteAccess.ClearPassword()
        RemoteAccess.ForgetAttempts()
        Self.Controller, Self.Bus = MakeController()
        Patch = patch.object(Bridge, "controller", Self.Controller)
        Patch.start()
        Self.addCleanup(Patch.stop)

    def SetPassword(Self, Value=Password):
        RemoteAccess.SetPassword(Value)

    def StateFirst(Self, Socket):
        """The first message on a socket that was let in: the state, as before."""
        Message = Socket.receive_json()
        Self.assertEqual(Message.get("type"), "state", Message)
        return Message

    def Status(Self, Socket):
        """The next status reply (states stream in between)."""
        for _ in range(400):
            Message = Socket.receive_json()
            if Message.get("type") == "status":
                return Message
        raise AssertionError("no status reply")


class RuleTests(unittest.TestCase):
    def TestOrigins(Self):
        Refusal = lambda Origin, Host: RemoteAccess.OriginRefusal({"origin": Origin, "host": Host} if Origin else {"host": Host})
        # Clancy's own page, however the bridge is reached; a program has no Origin.
        for Origin, Host in ((None, "127.0.0.1:8000"), ("http://127.0.0.1:5181", "127.0.0.1:8000"),
                             ("http://localhost:5181", "127.0.0.1:8000"), ("http://127.0.0.1:5180", "localhost:8000"),
                             ("http://[::1]:5181", "[::1]:8000"), (f"http://{PcLan}:5181", f"{PcLan}:8000"),
                             ("http://100.101.102.103:5181", "100.101.102.103:8000"), ("http://my-pc:5181", "my-pc:8000")):
            Self.assertIsNone(Refusal(Origin, Host), (Origin, Host))
        # Anything else: another site, another local web app (open-webui on 3000), the bridge's own /docs page, a
        # file:// page or sandboxed frame, a page on the LAN address asking the loopback bridge.
        for Origin, Host in (("http://evil.example:5181", "127.0.0.1:8000"), ("https://evil.example", "127.0.0.1:8000"),
                             ("http://127.0.0.1:3000", "127.0.0.1:8000"), ("http://my-pc:3000", "my-pc:8000"),
                             ("http://127.0.0.1:8000", "127.0.0.1:8000"), ("null", "127.0.0.1:8000"),
                             ("file://", "127.0.0.1:8000"), (f"http://{PcLan}:5181", "127.0.0.1:8000"),
                             ("http://127.0.0.1:99999", "127.0.0.1:8000")):
            Self.assertIsNotNone(Refusal(Origin, Host), (Origin, Host))
        with patch.dict(os.environ, {"SO101_DASHBOARD_PORTS": "5196"}):
            Self.assertIsNone(Refusal("http://127.0.0.1:5196", "127.0.0.1:8000"), "a test server's port can be added")
            Self.assertIn("5196", RemoteAccess.CorsOriginPattern())
        # A browser request with no Origin: an <img>, <script> or link on another page (another site, or another web
        # app on this PC, which is "same-site") is refused; an address typed in the address bar ("none") and the bridge's
        # own /docs page ("same-origin") are not. Programs send neither header.
        Fetch = lambda Site: RemoteAccess.OriginRefusal({"host": "127.0.0.1:8000", "sec-fetch-site": Site})
        for Site in ("cross-site", "same-site", "Cross-Site"):
            Self.assertIsNotNone(Fetch(Site), Site)
        for Site in ("none", "same-origin"):
            Self.assertIsNone(Fetch(Site), Site)

    def TestThisPc(Self):
        Local = lambda Client, Host, **Extra: RemoteAccess.IsLocal(Client, {"host": Host, **Extra})
        Self.assertTrue(Local("127.0.0.1", "127.0.0.1:8000"))
        Self.assertTrue(Local("127.0.0.1", "localhost:8000"))
        Self.assertTrue(Local("::1", "[::1]:8000"))
        Self.assertTrue(Local("::ffff:127.0.0.1", "127.0.0.1:8000"))
        # DNS rebinding: a page whose own name points at 127.0.0.1 arrives from loopback with that name as its Host.
        Self.assertFalse(Local("127.0.0.1", "evil.example:8000"))
        Self.assertFalse(Local("127.0.0.1", f"{PcLan}:8000"), "this PC by its LAN name needs the password like any device")
        # A local proxy carrying another device's request (tailscale serve, a reverse proxy) says so in its headers.
        Self.assertFalse(Local("127.0.0.1", "127.0.0.1:8000", **{"x-forwarded-for": "100.64.0.9"}))
        Self.assertFalse(Local("127.0.0.1", "127.0.0.1:8000", **{"tailscale-user-login": "someone@example.com"}))
        for Client in (Phone[0], "100.101.102.103", "fe80::1", "testclient", "", None):
            Self.assertFalse(Local(Client, "127.0.0.1:8000"), Client)


class LocalTests(Base):
    def TestThisPcNeedsNothing(Self):
        Self.SetPassword()  # a password set for other devices changes nothing here
        Client = LocalClient()
        for Origin in (None, "http://127.0.0.1:5181", "http://localhost:5181"):
            Self.assertEqual(Client.get("/ports", headers=Headers(Origin)).status_code, 200, Origin)
            Reply = Client.post("/motion", json={"type": "set_motion_speed", "percent": 40}, headers=Headers(Origin))
            Self.assertTrue(Reply.json()["ok"], Reply.json())
        Self.assertEqual(Self.Controller.MotionSpeedPercent, 40)
        Self.assertEqual(Client.get("/health").json()["access"], "local")
        with Client.websocket_connect("/ws", headers=Headers("http://127.0.0.1:5181")) as Socket:
            Self.StateFirst(Socket)  # no password asked, no extra message: the page sees what it always saw
            Socket.send_json({"type": "set_motion_speed", "percent": 60})
            Self.assertTrue(Self.Status(Socket)["ok"])
            Socket.send_json({"type": "auth"})  # harmless: already in
            Message = Socket.receive_json()
            while Message.get("type") != "access":
                Message = Socket.receive_json()
            Self.assertEqual((Message["ok"], Message["access"]), (True, "local"))
        Self.assertEqual(Self.Controller.MotionSpeedPercent, 60)

    def TestAProgramsEmergencyStopIsNotADeliberateDisconnect(Self):
        # The launcher (run-app.ps1) POSTs /emergency-stop when it stops: the next start connects as it always did.
        Self.assertTrue(LocalClient().post("/emergency-stop").json()["ok"])
        Self.assertIsNone(Self.Controller.DisconnectIntent)
        Self.assertFalse(Self.Bus.torque_enabled)


class CrossSiteTests(Base):
    def TestAnotherSitesPageIsRefused(Self):
        Self.SetPassword()
        Client = LocalClient()
        for Origin in ("http://evil.example:5181", "http://127.0.0.1:3000", "http://127.0.0.1:8000", "null"):
            for Method, Path, Body in (("get", "/ports", None), ("get", "/depthcam/status", None),
                                       ("post", "/motion", {"type": "set_motion_speed", "percent": 20}),
                                       ("post", "/emergency-stop", None)):
                Reply = getattr(Client, Method)(Path, headers=Headers(Origin, Key=Password), **({"json": Body} if Body else {}))
                Self.assertEqual(Reply.status_code, 403, (Origin, Path))
                Self.assertEqual(Reply.json()["access"], "cross_site")
            with Self.assertRaises(WebSocketDisconnect) as Refused:
                with Client.websocket_connect("/ws", headers=Headers(Origin)) as Socket:
                    Socket.receive_json()
            Self.assertEqual(Refused.exception.code, 1008)
        Self.assertNotEqual(Self.Controller.MotionSpeedPercent, 20, "no refused command ran")
        Self.assertTrue(Self.Bus.torque_enabled, "the refused emergency stop did not run either")

    def TestAnotherPagesImageOrLinkIsRefused(Self):
        # <img src="http://127.0.0.1:8000/selfcal/poses"> on any other page: no Origin, but the browser says where it
        # came from (GET /selfcal/poses computes and archives a calibration plan).
        Client = LocalClient()
        Calls = []
        with patch.object(Self.Controller, "selfcal_poses", lambda View: Calls.append(View) or {"ok": True}):
            for Site in ("cross-site", "same-site"):
                Reply = Client.get("/selfcal/poses", headers={"sec-fetch-site": Site, "sec-fetch-mode": "no-cors"})
                Self.assertEqual((Reply.status_code, Reply.json()["access"]), (403, "cross_site"), Site)
            Self.assertEqual(Calls, [], "no plan was made")
        # Typed into this PC's address bar, a read still works.
        Self.assertEqual(Client.get("/ports", headers={"sec-fetch-site": "none", "sec-fetch-mode": "navigate"}).status_code, 200)


class RemoteTests(Base):
    def TestWithNoPasswordSetOnlyHealthAnswers(Self):
        Client = Remote()
        Health = Client.get("/health", headers=Headers())
        Self.assertEqual(Health.status_code, 200)
        Self.assertEqual((Health.json()["access"], Health.json()["remote_access"]), ("open", "off"))
        for Path in ("/ports", "/depthcam/status", "/depthcam/frame.jpg", "/notepad", "/motion_trace"):
            Reply = Client.get(Path, headers=Headers(Key="anything at all"))
            Self.assertEqual((Reply.status_code, Reply.json()["access"]), (403, "remote_off"), Path)
        Reply = Client.post("/motion", json={"type": "set_motion_speed", "percent": 20}, headers=Headers(Key="x" * 12))
        Self.assertEqual(Reply.status_code, 403)
        with Client.websocket_connect("/ws", headers=Headers()) as Socket:
            Socket.send_json({"type": "auth", "password": Password})
            Message = Socket.receive_json()
            Self.assertEqual((Message["type"], Message["ok"], Message["access"]), ("access", False, "remote_off"))
            with Self.assertRaises(WebSocketDisconnect) as Closed:
                Socket.receive_json()
            Self.assertEqual(Closed.exception.code, 4403)
        Self.assertNotEqual(Self.Controller.MotionSpeedPercent, 20)

    def TestWithoutThePasswordNothingButHealth(Self):
        Self.SetPassword()
        Client = Remote()
        Self.assertEqual(Client.get("/health", headers=Headers()).json()["remote_access"], "password")
        for Key in (None, "", "wrong password"):
            Reply = Client.get("/ports", headers=Headers(Key=Key))
            Self.assertEqual(Reply.status_code, 401, Key)
            Self.assertEqual(Reply.json()["access"], "password_wrong" if Key else "password_required")
            Self.assertEqual(Client.post("/emergency-stop", headers=Headers(Key=Key)).status_code, 401)
            Reply = Client.post("/motion", json={"type": "set_motion_speed", "percent": 20}, headers=Headers(Key=Key))
            Self.assertEqual(Reply.status_code, 401)
        Self.assertNotEqual(Self.Controller.MotionSpeedPercent, 20)
        Self.assertTrue(Self.Bus.torque_enabled)
        # /ws: nothing (no state either) until the first message gives the password.
        for First, Code in (({"type": "auth"}, "password_required"), ({"type": "auth", "password": "wrong password"}, "password_wrong"),
                            ({"type": "set_motion_speed", "percent": 20}, "password_required")):
            with Client.websocket_connect("/ws", headers=Headers()) as Socket:
                Socket.send_json(First)
                Message = Socket.receive_json()
                Self.assertEqual((Message["type"], Message["ok"], Message["access"]), ("access", False, Code), First)
                with Self.assertRaises(WebSocketDisconnect) as Closed:
                    Socket.receive_json()
                Self.assertEqual(Closed.exception.code, 4401)
        Self.assertNotEqual(Self.Controller.MotionSpeedPercent, 20)
        # A socket that sends nothing is closed after AuthWaitS.
        with patch.object(RemoteAccess, "AuthWaitS", 0.2), Client.websocket_connect("/ws", headers=Headers()) as Socket:
            Self.assertEqual(Socket.receive_json()["access"], "password_required")

    def TestWithThePasswordEverythingWorks(Self):
        Self.SetPassword()
        Client = Remote()
        Self.assertEqual(Client.get("/ports", headers=Headers(Key=Password)).status_code, 200)
        Self.assertEqual(Client.get("/health", headers=Headers(Key=Password)).json()["access"], "open")
        Reply = Client.post("/motion", json={"type": "set_motion_speed", "percent": 30}, headers=Headers(Key=Password))
        Self.assertTrue(Reply.json()["ok"], Reply.json())
        Self.assertEqual(Self.Controller.MotionSpeedPercent, 30)
        with Client.websocket_connect("/ws", headers=Headers()) as Socket:
            Socket.send_json({"type": "auth", "password": Password})
            Message = Socket.receive_json()
            Self.assertEqual((Message["type"], Message["ok"], Message["access"]), ("access", True, "password"))
            Self.StateFirst(Socket)
            Socket.send_json({"type": "set_motion_speed", "percent": 50})
            Self.assertTrue(Self.Status(Socket)["ok"])
        Self.assertEqual(Self.Controller.MotionSpeedPercent, 50)
        # Over the tailnet alike, and a program may put the password in the handshake's header instead.
        Tailnet = Remote(("100.70.1.2", 40000), "100.101.102.103")
        with Tailnet.websocket_connect("/ws", headers={RemoteAccess.PasswordHeader: Password}) as Socket:
            Self.StateFirst(Socket)
        # The page's emergency stop works from there too, and counts as a deliberate disconnect.
        Self.assertTrue(Client.post("/emergency-stop", headers=Headers(Key=Password)).json()["ok"])
        Self.assertFalse(Self.Bus.torque_enabled)
        Self.assertEqual(Self.Controller.DisconnectIntent["by"], "another device")

    def TestAPageOnThePcsOwnLanNameOrThroughAProxyIsAnotherDevice(Self):
        Self.SetPassword()
        # DNS rebinding: evil.example made to point at 127.0.0.1; its page asks the bridge from loopback.
        Rebound = ClientAt(Bridge.app, base_url="http://evil.example:8000", client=("127.0.0.1", 50000))
        Self.assertEqual(Rebound.get("/ports", headers=Headers("http://evil.example:5181")).status_code, 401)
        with Rebound.websocket_connect("/ws", headers=Headers("http://evil.example:5181")) as Socket:
            Socket.send_json({"type": "auth"})
            Self.assertEqual(Socket.receive_json()["access"], "password_required")
        Proxied = LocalClient()
        Self.assertEqual(Proxied.get("/ports", headers={"x-forwarded-for": "100.64.0.9"}).status_code, 401)

    def TestWrongPasswordsLockAnAddressOutButARepeatedOneDoesNot(Self):
        Self.SetPassword()
        Client = Remote()
        # A page reconnecting with a stale password tries the same one again and again: one failure.
        for _ in range(8):
            Self.assertEqual(Client.get("/ports", headers=Headers(Key="the old password")).json()["access"], "password_wrong")
        Self.assertEqual(Client.get("/ports", headers=Headers(Key=Password)).status_code, 200)
        # The right password is now remembered as accepted (no scrypt next time): a lockout refuses it all the same.
        for Index in range(RemoteAccess.FailureLimit):
            Self.assertEqual(Client.get("/ports", headers=Headers(Key=f"guess number {Index}")).status_code, 401)
        Reply = Client.get("/ports", headers=Headers(Key=Password))
        Self.assertEqual((Reply.status_code, Reply.json()["access"]), (429, "locked_out"), "even the right one, meanwhile")
        Self.assertEqual(Remote(("192.168.1.31", 50000)).get("/ports", headers=Headers(Key=Password)).status_code, 200,
                         "another address is not locked out")
        # They age out.
        with patch.object(RemoteAccess.time, "monotonic", lambda: 10 ** 9):
            Self.assertEqual(Client.get("/ports", headers=Headers(Key=Password)).status_code, 200)

    def TestRequestsThroughAProxyShareOneLockout(Self):
        # 2026-10-08 access review: uvicorn's default rewrites a loopback client's address from X-Forwarded-For, so a
        # local process could rotate that header past the per-address lockout and be recorded as anyone. Every request
        # with a proxy header is one client now (and the launchers start uvicorn with --no-proxy-headers).
        Self.SetPassword()
        for Index in range(RemoteAccess.FailureLimit):
            Claimed = f"100.64.0.{Index + 1}"  # what uvicorn would have put in the scope from the header
            Reply = Remote((Claimed, 50000)).get("/ports", headers=Headers(Key=f"guess number {Index}", **{"x-forwarded-for": Claimed}))
            Self.assertEqual(Reply.status_code, 401)
        Reply = Remote(("100.64.0.99", 50000)).get("/ports", headers=Headers(Key=Password, **{"x-forwarded-for": "100.64.0.99"}))
        Self.assertEqual((Reply.status_code, Reply.json()["access"]), (429, "locked_out"))
        with LocalClient().websocket_connect("/ws", headers={"x-forwarded-for": "100.64.0.7"}) as Socket:
            Socket.send_json({"type": "auth", "password": Password})
            Self.assertEqual(Socket.receive_json()["access"], "locked_out", "the socket's own check keys it alike")
        Self.assertEqual(Remote().get("/ports", headers=Headers(Key=Password)).status_code, 200, "a device not behind a proxy")
        Self.assertEqual(RemoteAccess.ClientName("100.64.0.99", {"x-forwarded-for": "100.64.0.99"}),
                         ("proxied (claims 100.64.0.99)", "proxied"))
        Self.assertEqual(RemoteAccess.ClientName("192.168.1.30", {}), ("192.168.1.30", "192.168.1.30"))

    def TestEveryLauncherStartsTheBridgeWithoutProxyHeaders(Self):
        App = Path(__file__).resolve().parent.parent / "AI Controlled WebApp"
        for File in ("vite.config.ts", "scripts/RestartBridge.ps1", "scripts/run-app.ps1"):
            Text = (App / File).read_text(encoding="utf-8")
            Self.assertIn('"bridge_server:app"', Text, File)
            Self.assertIn('"--no-proxy-headers"', Text, File)

    def TestThePageCanReadARefusalAndSendItsPassword(Self):
        Self.SetPassword()
        Client = Remote()
        Preflight = Client.options("/ports", headers={"origin": PhoneOrigin, "access-control-request-method": "GET",
                                                      "access-control-request-headers": RemoteAccess.PasswordHeader})
        Self.assertEqual(Preflight.status_code, 200)
        Self.assertEqual(Preflight.headers["access-control-allow-origin"], PhoneOrigin)
        Self.assertIn(RemoteAccess.PasswordHeader, Preflight.headers["access-control-allow-headers"].lower())
        Refused = Client.get("/ports", headers=Headers())
        Self.assertEqual(Refused.headers.get("access-control-allow-origin"), PhoneOrigin, "the page can read why")
        # Tailscale's 100.x pages were not even allowed to read replies before (only 192.168.x and 10.x).
        Tailnet = Remote(("100.70.1.2", 40000), "100.101.102.103")
        Reply = Tailnet.get("/ports", headers=Headers("http://100.101.102.103:5181", Key=Password))
        Self.assertEqual(Reply.headers.get("access-control-allow-origin"), "http://100.101.102.103:5181")
        # CORS admits Clancy's page on any host (a LAN or tailnet address, a name): it is not the guard. Another site's
        # page gets only its refusal to read.
        Reply = Remote().get("/ports", headers=Headers("http://evil.example:5181", Key=Password))
        Self.assertEqual((Reply.status_code, Reply.json()["access"]), (403, "cross_site"))


class IntentTests(Base):
    def setUp(Self):
        super().setUp()
        Self.Made = []

        def MakeBus(Port):
            Bus = ConnectingBus()
            for Name, Servo in Self.Controller.servos.items():
                Bus.position[Name] = Servo.present_raw
            Self.Made.append(Port)
            return Bus
        Patch = patch.object(Bridge, "_make_bus", MakeBus)
        Patch.start()
        Self.addCleanup(Patch.stop)

    def TestADeliberateDisconnectHoldsUntilAnExplicitConnect(Self):
        with LocalClient().websocket_connect("/ws", headers=Headers("http://127.0.0.1:5181")) as Socket:
            Self.StateFirst(Socket)
            Socket.send_json({"type": "disconnect"})
            Self.assertTrue(Self.Status(Socket)["ok"])
            State = Socket.receive_json()
            Self.assertEqual(State["disconnected_on_purpose"]["by"], "this PC")
            Self.assertFalse(State["arm_connected"])
            # Another page opening (another browser, profile or device) and connecting by itself: refused.
            Socket.send_json({"type": "connect", "port": "FAKE"})
            Reply = Self.Status(Socket)
            Self.assertFalse(Reply["ok"])
            Self.assertIn("disconnected on purpose", Reply["text"])
            Self.assertEqual(Self.Made, [], "no serial port was opened")
            # Someone presses Connect.
            Socket.send_json({"type": "connect", "port": "FAKE", "explicit": True})
            Self.assertTrue(Self.Status(Socket)["ok"])
            State = Socket.receive_json()
            Self.assertTrue(State["arm_connected"])
            Self.assertIsNone(State["disconnected_on_purpose"])
        Self.assertEqual(Self.Made, ["FAKE"])

    def TestAPagesEmergencyStopHoldsToo(Self):
        with LocalClient().websocket_connect("/ws", headers=Headers("http://127.0.0.1:5181")) as Socket:
            Self.StateFirst(Socket)
            Socket.send_json({"type": "emergency_stop"})
            Self.assertTrue(Self.Status(Socket)["ok"])
        Self.assertIsNotNone(Self.Controller.DisconnectIntent)
        Self.Controller.DisconnectIntent = None
        Self.assertTrue(LocalClient().post("/emergency-stop", headers=Headers("http://127.0.0.1:5181")).json()["ok"])
        Self.assertIsNotNone(Self.Controller.DisconnectIntent, "the page's HTTP emergency stop (it has an Origin)")
        Ok, Text = Self.Controller.handle_command({"type": "connect", "port": "FAKE"})
        Self.assertFalse(Ok)
        Self.assertIn("disconnected on purpose", Text)

    def TestAnotherDeviceNeverConnectsByItself(Self):
        Self.SetPassword()
        Self.Controller.disconnect(disable_torque=True)  # the arm is off (not on purpose: e.g. the bridge just started)
        Self.assertIsNone(Self.Controller.DisconnectIntent)
        Client = Remote()
        Reply = Client.post("/motion", json={"type": "connect", "port": "FAKE"}, headers=Headers(Key=Password)).json()
        Self.assertFalse(Reply["ok"])
        Self.assertIn("only when its Connect button is pressed", Reply["error"])
        with Client.websocket_connect("/ws", headers=Headers()) as Socket:
            Socket.send_json({"type": "auth", "password": Password})
            Self.assertTrue(Socket.receive_json()["ok"])
            Self.StateFirst(Socket)
            Socket.send_json({"type": "connect", "port": "FAKE"})
            Self.assertIn("only when its Connect button is pressed", Self.Status(Socket)["text"])
            Self.assertEqual(Self.Made, [])
            Self.assertFalse(Self.Controller.torque_enabled, "no torque came on")
            Socket.send_json({"type": "connect", "port": "FAKE", "explicit": True})
            Self.assertTrue(Self.Status(Socket)["ok"])
        Self.assertEqual(Self.Made, ["FAKE"])
        Self.assertTrue(Self.Controller.torque_enabled, "Connect pressed: the arm comes out of connect holding")
        # This PC's page still connects by itself when nobody disconnected on purpose (its experience is unchanged).
        Self.Controller.disconnect(disable_torque=True)
        Self.assertTrue(Self.Controller.handle_command({"type": "connect", "port": "FAKE"}, "local")[0])

    def TestTheIntentNeverWaitsOnTheMotorLockAndReachesDiskAfterTheCoast(Self):
        # The /ws loop sets it in the event loop itself: a connect holding the motor lock (up to a second) must not
        # stall every socket, and an emergency stop must not wait for a file to be written.
        Held = threading.Event()
        Release = threading.Event()

        def Hold():
            with Self.Controller.lock:
                Held.set()
                Release.wait(5)
        Holder = threading.Thread(target=Hold)
        Holder.start()
        Held.wait(5)
        try:
            Started = time.monotonic()
            Self.Controller.SetDisconnectIntent(True, "local", Keep=False)
            # Waiting for the lock would take the 5 s the holder keeps it; half of that tells the two apart on a busy PC.
            Self.assertLess(time.monotonic() - Started, 2.5, "set without the motor lock")
            Self.assertIsNotNone(Self.Controller.ConnectRefusal({"type": "connect"}))
        finally:
            Release.set()
            Holder.join()
        Self.Controller.DisconnectIntent = None
        Written = []
        Self.Controller.IntentSink = lambda Intent: Written.append((Intent is not None, Self.Bus.torque_enabled))
        with LocalClient().websocket_connect("/ws", headers=Headers("http://127.0.0.1:5181")) as Socket:
            Self.StateFirst(Socket)
            Socket.send_json({"type": "emergency_stop"})
            Self.assertTrue(Self.Status(Socket)["ok"])
        Self.assertEqual(Written, [(True, False)], "written once, after the torque went off")
        Written.clear()
        Self.Controller.handle_command({"type": "disconnect"})
        Self.assertEqual(Written, [(True, False)])

    def TestTheIntentIsKeptOnDiskAcrossARestart(Self):
        File = Path(os.environ["SO101_CONNECT_INTENT_FILE"])
        File.unlink(missing_ok=True)
        Self.Controller.IntentSink = RemoteAccess.SaveIntent
        Self.Controller.handle_command({"type": "disconnect"}, "password")
        Self.assertEqual(RemoteAccess.LoadIntent()["by"], "another device")
        # A restarted bridge (its lifespan) loads it: still no automatic connect anywhere.
        Restarted, _ = MakeController()
        Events = []

        class Loop:
            def start(Self): Events.append("loop start")
            def stop(Self): Events.append("loop stop")

        class Camera:
            def Start(Self): pass
            def Stop(Self): pass

        async def Run():
            with patch.object(Bridge, "controller", Restarted), patch.object(Bridge, "control_loop", Loop()), \
                    patch.object(Bridge, "depth_camera", Camera()), patch.object(Bridge, "AppendBridgeEvent", Events.append), \
                    patch.object(Bridge, "RaiseProcessPriority", lambda: "normal"), \
                    patch.object(Bridge.RouteChecks, "Start", lambda: None), patch.object(RemoteAccess, "EventSink", None):
                async with Bridge._lifespan(Bridge.app):
                    Self.assertEqual(Restarted.DisconnectIntent["by"], "another device")
                    Self.assertIn("disconnected on purpose", Restarted.ConnectRefusal({"type": "connect"}))
                    Self.assertTrue(Restarted.handle_command({"type": "connect", "port": "FAKE", "explicit": True})[0])
                    Self.assertIsNone(RemoteAccess.LoadIntent(), "an explicit Connect clears it on disk too")
        asyncio.run(Run())
        Self.assertEqual(json.loads(File.read_text(encoding="utf-8"))["DisconnectedOnPurpose"], False)


class SetterTests(unittest.TestCase):
    def setUp(Self):
        RemoteAccess.ClearPassword()
        RemoteAccess.ForgetAttempts()

    def TestTheFileHoldsAHashNotThePassword(Self):
        File = RemoteAccess.SetPassword(Password)
        Text = File.read_text(encoding="utf-8")
        Self.assertNotIn(Password, Text)
        Self.assertNotIn(Password.replace(" ", ""), Text)
        Self.assertEqual(json.loads(Text)["Kdf"], "scrypt")
        Self.assertEqual(RemoteAccess.CheckPassword(Password, "a"), (True, "ok"))
        Self.assertEqual(RemoteAccess.CheckPassword(Password.upper(), "a"), (False, "password_wrong"))
        for Bad in ("short", " leading space x", "trailing space x ", "naïve passphrase", "x" * 129):
            with Self.assertRaises(ValueError):
                RemoteAccess.SetPassword(Bad)

    def TestANewPasswordWorksWithoutARestart(Self):
        RemoteAccess.SetPassword(Password)
        Self.assertTrue(RemoteAccess.CheckPassword(Password, "a")[0])
        os.utime(RemoteAccess.PasswordPath(), ns=(1, 1))  # a different mtime even within the clock's tick
        RemoteAccess.SetPassword("a different password")
        Self.assertEqual(RemoteAccess.CheckPassword(Password, "a"), (False, "password_wrong"))
        Self.assertTrue(RemoteAccess.CheckPassword("a different password", "a")[0])
        RemoteAccess.ClearPassword()
        Self.assertEqual(RemoteAccess.CheckPassword("a different password", "a"), (False, "remote_off"))

    def TestTheCommandLine(Self):
        Out = io.StringIO()
        with redirect_stdout(Out):
            Self.assertEqual(SetRemotePassword.Main(["--generate"]), 0)
        Generated = Out.getvalue().split("Remote password: ")[1].split()[0]
        Self.assertRegex(Generated, r"^[a-z2-9]{4}-[a-z2-9]{4}-[a-z2-9]{4}-[a-z2-9]{4}$")
        Self.assertTrue(RemoteAccess.CheckPassword(Generated, "a")[0])
        with redirect_stdout(io.StringIO()) as Status:
            SetRemotePassword.Main(["--status"])
        Self.assertIn("is set", Status.getvalue())
        with patch.object(SetRemotePassword.getpass, "getpass", side_effect=["my own passphrase", "my own passphrase"]), \
                redirect_stdout(io.StringIO()):
            Self.assertEqual(SetRemotePassword.Main([]), 0)
        Self.assertTrue(RemoteAccess.CheckPassword("my own passphrase", "a")[0])
        with patch.object(SetRemotePassword.getpass, "getpass", side_effect=["my own passphrase", "a typo in it"]), \
                redirect_stdout(io.StringIO()), patch("sys.stderr", io.StringIO()):
            Self.assertEqual(SetRemotePassword.Main([]), 2)
        with redirect_stdout(io.StringIO()):
            Self.assertEqual(SetRemotePassword.Main(["--off"]), 0)
        Self.assertFalse(RemoteAccess.PasswordSet())

    def TestTheMachineFilesAreNeverCommitted(Self):
        Ignore = (Path(__file__).resolve().parent.parent / ".gitignore").read_text(encoding="utf-8")
        Self.assertIn("back end/RemotePassword.json", Ignore)
        Self.assertIn("back end/ConnectIntent.json", Ignore)


Cases = (RuleTests, LocalTests, CrossSiteTests, RemoteTests, IntentTests, SetterTests)


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestRemoteAccess` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return unittest.TestSuite(Prefixed.loadTestsFromTestCase(Case) for Case in Cases)


def tearDownModule():
    RemoteAccess.ClearPassword()
    shutil.rmtree(Scratch, ignore_errors=True)


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    try:
        Result = unittest.TextTestRunner(verbosity=2).run(unittest.TestSuite(Loader.loadTestsFromTestCase(Case) for Case in Cases))
    finally:
        tearDownModule()
    raise SystemExit(not Result.wasSuccessful())
