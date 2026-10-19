"""Who may use the bridge: this PC as before, any other device only with the remote password.

The bridge listens on every interface so that a phone or laptop on the LAN or the tailnet can open Clancy. Until
2026-10-07 that also meant anyone who could reach port 8000 could connect the arm, turn its torque on and move it
(/ws and POST /motion took any command from anyone), and a web page from any site open in a browser on this PC could
do the same through ws://127.0.0.1:8000/ws (audit 2 H2, audit 1 S5). Aaran's decision (2026-10-07): commands only from
this PC by default, other devices with a password, and nothing connects the arm or turns its torque on by itself from
another device. The rule, for every HTTP request and WebSocket (Middleware):

1. A browser request (it carries an Origin) must come from Clancy's own page: http(s), on the host the request names
   (its Host header) and on a dashboard port (5180, 5181; SO101_DASHBOARD_PORTS adds test servers' ports). A page from
   any other site, a file:// page or a sandboxed frame ("null") is refused, password or not. A browser request with no
   Origin (an <img>, <script> or link another page points at the bridge: GET /selfcal/poses writes a plan) still says
   where it came from in Sec-Fetch-Site: only "none" (typed in the address bar) and "same-origin" (the bridge's own
   /docs) pass; Clancy's page reaches the bridge only by fetch and WebSocket, which always send an Origin. Programs
   (neither header) go on to rule 2.
2. This PC needs nothing: a client on the loopback interface that names the bridge by a loopback name (127.0.0.1,
   localhost, ::1) and carries no proxy header. The name matters: a web page whose own domain is pointed at 127.0.0.1
   (DNS rebinding) reaches the bridge from loopback with that domain as its Host, and passes rule 1. A local proxy
   (tailscale serve, netsh portproxy) would make every device look local; its headers give it away.
3. Any other device needs the remote password, set on this PC with SetRemotePassword.py (RemotePassword.json beside
   this file, git-ignored, holds only a salted scrypt hash): the X-Clancy-Password header on HTTP, the first message
   {"type": "auth", "password": ...} on /ws (a browser cannot put headers on a WebSocket). Reads need it too: the depth
   camera films the room, and the arm's state, the notepad and the serial ports are nobody else's business. Only
   GET /health (versions and liveness, which another device's page uses to say "the bridge is up, enter the password")
   and CORS preflights stay open. With no password set, other devices get nothing else at all. Five different wrong
   passwords from one address within 10 minutes lock that address out, the right password too, until they age out
   (the same wrong password tried again, as a page with a stale one reconnecting does, counts once).

What a command may do is decided elsewhere: the bridge refuses a connect that is not an explicit Connect click from
another device, and from anywhere while the arm was disconnected on purpose (BridgeController.ConnectRefusal; the
intent is kept on disk by LoadIntent / SaveIntent below, so a restarted bridge keeps it too).
"""

from __future__ import annotations

import asyncio
import base64
import collections
import hashlib
import hmac
import ipaddress
import json
import os
import secrets
import sys
import threading
import time
from pathlib import Path
from typing import Any, Callable
from urllib.parse import urlsplit

BackendRoot = Path(__file__).resolve().parent
# The header another device's page sends its password in, and the key Middleware leaves its verdict under in the
# ASGI scope: "local" (this PC), "password" (another device that gave it), "open" (GET /health and preflights from
# another device) or "pending" (another device's /ws, which must send the password as its first message).
PasswordHeader = "x-clancy-password"
AccessKey = "clancy_access"
DefaultDashboardPorts = (5180, 5181)
OpenPaths = frozenset({"/health"})
ProxyHeaders = ("forwarded", "x-forwarded-for", "x-forwarded-host", "x-real-ip", "via", "tailscale-user-login")
MinimumPasswordLength = 10
MaximumCandidateLength = 256
FailureWindowS = 600.0
FailureLimit = 5
# How long another device's /ws may take to send its password before it is closed.
AuthWaitS = 5.0
# A refusal per address and reason is printed (and recorded) at most this often.
RefusalNoteS = 600.0
# scrypt (RFC 7914): about 50 ms per check here, 16 MiB of memory. Accepted passwords are remembered (Verified), so
# only the first request of each device pays it; the dev server (scripts/RemoteAccess.ts) checks the same file alike.
ScryptN, ScryptR, ScryptP, ScryptBytes = 2 ** 14, 8, 1, 32

Refusals = {
    "cross_site": (403, "Refused: this request came from a web page that is not Clancy's own."),
    "remote_off": (403, "Remote access is off: only the robot's PC can use the bridge until a remote password is set "
                        "there (back end/SetRemotePassword.py; README, Using Clancy from another device)."),
    "password_required": (401, "This device needs the arm password set on the robot's PC: enter it in Clancy."),
    "password_wrong": (401, "Wrong arm password."),
    "locked_out": (429, "Too many wrong arm passwords from this device: wait 10 minutes, then try again."),
}
# The close code another device's /ws gets with each refusal (4000-4999 are the application's own).
CloseCodes = {"password_required": 4401, "password_wrong": 4401, "remote_off": 4403, "locked_out": 4429}

_Lock = threading.Lock()
_FileCache: dict[str, Any] = {"Key": None, "Record": None}
_Verified: set[str] = set()
_Failures: dict[str, collections.deque] = {}
_RefusalNotes: dict[tuple[str, str], float] = {}
# Where refusals and accepted devices are recorded besides stderr (the running server points it at the bridge's
# events log, bridge_events.jsonl); None offline and in tests.
EventSink: Callable[[dict[str, Any]], None] | None = None


def PasswordPath() -> Path:
    return Path(os.environ.get("SO101_REMOTE_PASSWORD_FILE") or BackendRoot / "RemotePassword.json")


def IntentPath() -> Path:
    return Path(os.environ.get("SO101_CONNECT_INTENT_FILE") or BackendRoot / "ConnectIntent.json")


def DashboardPorts() -> frozenset[int]:
    """The ports Clancy's page is served from: 5180 and 5181, and any listed in SO101_DASHBOARD_PORTS (test servers)."""
    Ports = set(DefaultDashboardPorts)
    for Item in os.environ.get("SO101_DASHBOARD_PORTS", "").replace(";", ",").split(","):
        if Item.strip().isdigit():
            Ports.add(int(Item.strip()))
    return frozenset(Ports)


def CorsOriginPattern() -> str:
    """Clancy's page on any host, on a dashboard port: CORS lets it read replies; Middleware decides who may use them."""
    return r"https?://(\[[0-9A-Fa-f:.]+\]|[^/:\[\]@]+):(" + "|".join(str(Port) for Port in sorted(DashboardPorts())) + ")"


def HostName(Value: str | None) -> str | None:
    """The lowercased host of a Host header ("127.0.0.1:8000", "[::1]:8000", "pc"), without port or brackets."""
    if not Value:
        return None
    try:
        return urlsplit("//" + Value.strip()).hostname
    except ValueError:
        return None


def IsLoopbackName(Host: str | None) -> bool:
    if not Host:
        return False
    if Host == "localhost":
        return True
    try:
        return ipaddress.ip_address(Host).is_loopback
    except ValueError:
        return False


def IsLoopbackClient(Address: str | None) -> bool:
    """Whether a client's address is this PC's loopback interface (a test client named "testclient" is not)."""
    try:
        Ip = ipaddress.ip_address(Address or "")
    except ValueError:
        return False
    if Ip.version == 6 and Ip.ipv4_mapped is not None:
        Ip = Ip.ipv4_mapped
    return Ip.is_loopback


def OriginRefusal(Headers: dict[str, str]) -> str | None:
    """Rule 1: None for a program, a page of Clancy's own or an address typed into a browser, else why not."""
    Origin = Headers.get("origin")
    if Origin is None:
        Site = Headers.get("sec-fetch-site")
        if Site is None or Site.strip().lower() in ("none", "same-origin"):
            return None
        return f"a browser request from another page ({Site}) without an Origin"
    try:
        Parts = urlsplit(Origin.strip())
        Scheme, Host, Port = Parts.scheme, Parts.hostname, Parts.port
    except ValueError:
        return f"origin {Origin!r} cannot be read"
    if Scheme not in ("http", "https") or not Host:
        return f"origin {Origin!r} is not a web page's"
    Target = HostName(Headers.get("host"))
    if not (Host == Target or (IsLoopbackName(Host) and IsLoopbackName(Target))):
        return f"origin {Origin!r} is another site than the host it asked ({Target})"
    if (Port or (443 if Scheme == "https" else 80)) not in DashboardPorts():
        return f"origin {Origin!r} is not on a Clancy dashboard port ({', '.join(map(str, sorted(DashboardPorts())))})"
    return None


def IsLocal(Client: str | None, Headers: dict[str, str]) -> bool:
    """Rule 2: this PC itself, asking by a loopback name, not through a proxy."""
    return (IsLoopbackClient(Client) and IsLoopbackName(HostName(Headers.get("host")))
            and not any(Name in Headers for Name in ProxyHeaders))


# --- The remote password ---------------------------------------------------------------------------------------------

def ValidPasswordProblem(Password: str) -> str | None:
    """Why a new password cannot be used, or None. Printable ASCII only, so a browser can send it in a header."""
    if len(Password) < MinimumPasswordLength:
        return f"use at least {MinimumPasswordLength} characters"
    if len(Password) > 128:
        return "use at most 128 characters"
    if any(not (32 <= ord(Character) < 127) for Character in Password):
        return "use letters, digits, spaces and punctuation only (printable ASCII)"
    if Password != Password.strip():
        return "no spaces at the start or end"
    return None


def _Hash(Password: str, Salt: bytes, N: int, R: int, P: int, Length: int) -> bytes:
    return hashlib.scrypt(Password.encode("utf-8"), salt=Salt, n=N, r=R, p=P, dklen=Length)


def _WriteAtomically(File: Path, Text: str) -> None:
    File.parent.mkdir(parents=True, exist_ok=True)
    Temporary = File.with_name(f"{File.name}.{os.getpid()}.tmp")
    with open(Temporary, "w", encoding="utf-8") as Out:
        Out.write(Text)
        Out.flush()
        os.fsync(Out.fileno())
    os.replace(Temporary, File)


def SetPassword(Password: str, File: Path | None = None) -> Path:
    """Store a new remote password's salted scrypt hash (never the password). Raises ValueError for a bad one."""
    Problem = ValidPasswordProblem(Password)
    if Problem:
        raise ValueError(Problem)
    File = File or PasswordPath()
    Salt = secrets.token_bytes(16)
    Record = {"Version": 1, "Kdf": "scrypt", "N": ScryptN, "R": ScryptR, "P": ScryptP,
              "Salt": base64.b64encode(Salt).decode("ascii"),
              "Hash": base64.b64encode(_Hash(Password, Salt, ScryptN, ScryptR, ScryptP, ScryptBytes)).decode("ascii"),
              "SetAt": time.strftime("%Y-%m-%dT%H:%M:%S"),
              "Note": "The remote password's scrypt hash (back end/SetRemotePassword.py). Machine-local: never commit it."}
    _WriteAtomically(File, json.dumps(Record, indent=2) + "\n")
    return File


def ClearPassword(File: Path | None = None) -> bool:
    """Remove the remote password: other devices are refused again. False when none was set."""
    File = File or PasswordPath()
    try:
        File.unlink()
        return True
    except FileNotFoundError:
        return False


def LoadPassword(File: Path | None = None) -> dict[str, Any] | None:
    """The stored hash record, read again whenever the file changes (no restart after SetRemotePassword.py)."""
    File = File or PasswordPath()
    try:
        Stat = File.stat()
        Key = (str(File), Stat.st_mtime_ns, Stat.st_size)
    except OSError:
        return None
    with _Lock:
        if _FileCache["Key"] == Key:
            return _FileCache["Record"]
    try:
        Record = json.loads(File.read_text(encoding="utf-8"))
        Record = {"N": int(Record["N"]), "R": int(Record["R"]), "P": int(Record["P"]),
                  "Salt": base64.b64decode(Record["Salt"]), "Hash": base64.b64decode(Record["Hash"])}
        if Record["N"] < 2 ** 10 or not Record["Hash"]:
            raise ValueError("weak or empty hash")
    except (OSError, ValueError, KeyError, TypeError) as Exc:
        print(f"[remote-access] {File} is unreadable ({Exc}): remote access stays off", file=sys.stderr)
        Record = None
    with _Lock:
        _FileCache.update(Key=Key, Record=Record)
    return Record


def PasswordSet() -> bool:
    return LoadPassword() is not None


def CheckPassword(Candidate: str | None, Client: str) -> tuple[bool, str]:
    """Whether Candidate is the remote password: (True, "ok") or (False, why) with why one of Refusals' keys."""
    Record = LoadPassword()
    if Record is None:
        return False, "remote_off"
    if not Candidate or not isinstance(Candidate, str):
        return False, "password_required"
    Candidate = Candidate[:MaximumCandidateLength]  # longer is wrong anyway; scrypt need not chew on megabytes
    Now = time.monotonic()
    Attempt = hashlib.sha256(Record["Hash"] + b"\0" + Candidate.encode("utf-8", "replace")).hexdigest()
    with _Lock:
        # A locked-out address is refused before anything is checked, the right password too, until its failures age
        # out: the same answer whether or not another device already used that password.
        Recent = _Failures.setdefault(Client, collections.deque())
        while Recent and Now - Recent[0][0] > FailureWindowS:
            Recent.popleft()
        if len({Tried for _, Tried in Recent}) >= FailureLimit:
            return False, "locked_out"
        if Attempt in _Verified:
            _Failures.pop(Client, None)
            return True, "ok"
    Ok = hmac.compare_digest(_Hash(Candidate, Record["Salt"], Record["N"], Record["R"], Record["P"], len(Record["Hash"])),
                             Record["Hash"])
    with _Lock:
        if Ok:
            _Verified.add(Attempt)
            _Failures.pop(Client, None)
            return True, "ok"
        _Failures.setdefault(Client, collections.deque()).append((Now, Attempt))
    return False, "password_wrong"


def ForgetAttempts() -> None:
    """Drop the accepted-password memory and every address's failures (tests; a new password needs no call)."""
    with _Lock:
        _Verified.clear()
        _Failures.clear()
        _RefusalNotes.clear()
        _FileCache.update(Key=None, Record=None)


def Note(Kind: str, Client: str, Code: str, Detail: str) -> None:
    """Print, and record through EventSink, a refusal or a device let in: once per address and reason per 10 minutes."""
    Now = time.monotonic()
    with _Lock:
        if Now - _RefusalNotes.get((Client, Code), -RefusalNoteS) < RefusalNoteS:
            return
        _RefusalNotes[(Client, Code)] = Now
    print(f"[remote-access] {Kind} {Client}: {Code} ({Detail})", file=sys.stderr)
    if EventSink is not None:
        try:
            Wall = time.time()
            EventSink({"kind": "remote_access", "reason": f"{Kind}: {Code}", "joint": None,
                       "details": {"client": Client, "code": Code, "detail": Detail}, "t": round(Wall, 3),
                       "time": time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(Wall)) + f".{int(Wall % 1 * 1000):03d}Z"})
        except Exception as Exc:
            print(f"[remote-access] could not record: {Exc}", file=sys.stderr)


# --- The ASGI middleware ---------------------------------------------------------------------------------------------

def _Headers(Scope: dict[str, Any]) -> dict[str, str]:
    return {Key.decode("latin-1").lower(): Value.decode("latin-1") for Key, Value in Scope.get("headers", [])}


async def _RefuseHttp(Send, Code: str, Detail: str | None = None) -> None:
    Status, Text = Refusals[Code]
    Body = json.dumps({"ok": False, "error": Text + (f" ({Detail})" if Detail else ""), "access": Code}).encode("utf-8")
    await Send({"type": "http.response.start", "status": Status,
                "headers": [(b"content-type", b"application/json"), (b"content-length", str(len(Body)).encode("ascii")),
                            (b"cache-control", b"no-store")]})
    await Send({"type": "http.response.body", "body": Body})


async def _RefuseSocket(Receive, Send) -> None:
    """Refuse a WebSocket handshake (the server answers HTTP 403): nothing is accepted, nothing is sent."""
    Message = await Receive()
    if Message.get("type") == "websocket.connect":
        await Send({"type": "websocket.close", "code": 1008})


def ClientName(Client: str, Headers: dict[str, str]) -> tuple[str, str]:
    """(how a client is named in the record, its key for the lockout). A request through a proxy claims whatever
    address it likes: uvicorn rewrites a loopback client's from X-Forwarded-For unless started with --no-proxy-headers
    (the launchers start it so), and a local process rotating that header dodged the per-address lockout and was
    recorded as whom it named (2026-10-08 access review). Every request carrying a proxy header is one client for both,
    and never local (IsLocal)."""
    if any(Name in Headers for Name in ProxyHeaders):
        return f"proxied (claims {Client or 'nothing'})"[:80], "proxied"
    return Client, Client


class Middleware:
    """Applies the rules above to every request before any route sees it (see the module's docstring)."""

    def __init__(Self, App) -> None:
        Self.App = App

    async def __call__(Self, Scope, Receive, Send) -> None:
        if Scope["type"] not in ("http", "websocket"):
            await Self.App(Scope, Receive, Send)
            return
        Headers = _Headers(Scope)
        Client = (Scope.get("client") or ("", 0))[0] or ""
        Name, Key = ClientName(Client, Headers)
        Socket = Scope["type"] == "websocket"
        Problem = OriginRefusal(Headers)
        if Problem:
            Note("refused", Name, "cross_site", f"{Scope.get('path')}: {Problem}")
            await (_RefuseSocket(Receive, Send) if Socket else _RefuseHttp(Send, "cross_site", Problem))
            return
        if IsLocal(Client, Headers):
            Scope[AccessKey] = "local"
            await Self.App(Scope, Receive, Send)
            return
        Method = Scope.get("method", "GET")
        if not Socket and (Method == "OPTIONS" or (Method in ("GET", "HEAD") and Scope.get("path") in OpenPaths)):
            Scope[AccessKey] = "open"
            await Self.App(Scope, Receive, Send)
            return
        Password = Headers.get(PasswordHeader)
        if Socket and Password is None:
            Scope[AccessKey] = "pending"  # the endpoint takes it from the first message (bridge_server.AuthenticateSocket)
            await Self.App(Scope, Receive, Send)
            return
        Ok, Code = await asyncio.to_thread(CheckPassword, Password, Key)
        if not Ok:
            Note("refused", Name, Code, f"{Method if not Socket else 'WebSocket'} {Scope.get('path')}")
            await (_RefuseSocket(Receive, Send) if Socket else _RefuseHttp(Send, Code))
            return
        Note("let in", Name, "password", f"{Method if not Socket else 'WebSocket'} {Scope.get('path')}")
        Scope[AccessKey] = "password"
        await Self.App(Scope, Receive, Send)


# --- Whether the arm was disconnected on purpose (audit 2 M2) --------------------------------------------------------

def LoadIntent(File: Path | None = None) -> dict[str, Any] | None:
    """The kept "disconnected on purpose" ({"at", "by"}), or None: never set, cleared by an explicit Connect, unreadable."""
    try:
        Record = json.loads((File or IntentPath()).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    if not isinstance(Record, dict) or Record.get("DisconnectedOnPurpose") is not True:
        return None
    return {"at": str(Record.get("At") or ""), "by": str(Record.get("By") or "")}


def SaveIntent(Intent: dict[str, Any] | None, File: Path | None = None) -> None:
    Record = ({"DisconnectedOnPurpose": True, "At": Intent.get("at"), "By": Intent.get("by")} if Intent
              else {"DisconnectedOnPurpose": False})
    Record["Note"] = ("Kept by the bridge (RemoteAccess.SaveIntent): after Disconnect or Emergency Shutoff no page "
                      "connects the arm by itself until someone presses Connect. Machine-local: never commit it.")
    _WriteAtomically(File or IntentPath(), json.dumps(Record, indent=2) + "\n")
