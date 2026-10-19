"""Set the password other devices (a phone, a laptop, over the LAN or Tailscale) need to use the arm's bridge.

  python SetRemotePassword.py              asks for a new password twice (not shown as you type)
  python SetRemotePassword.py --generate   makes a strong one, stores it and prints it once
  python SetRemotePassword.py --off        removes it: only this PC can use the bridge again
  python SetRemotePassword.py --status     says whether one is set

This PC never needs it. RemotePassword.json beside this file keeps only a salted scrypt hash and is git-ignored;
SO101_REMOTE_PASSWORD_FILE points at another file (tests). The bridge and Clancy's dev server read it again on their
next request, so no restart is needed; every device then enters the new password once (RemoteAccess.py has the rules).
"""

from __future__ import annotations

import argparse
import getpass
import secrets
import sys

import RemoteAccess

# No 0/o, 1/l/i: easy to read off this screen and type on a phone. 16 of 31 symbols is about 79 bits.
GeneratedAlphabet = "abcdefghjkmnpqrstuvwxyz23456789"


def Generate() -> str:
    Characters = "".join(secrets.choice(GeneratedAlphabet) for _ in range(16))
    return "-".join(Characters[Index:Index + 4] for Index in range(0, 16, 4))


def Main(Argv: list[str] | None = None) -> int:
    Parser = argparse.ArgumentParser(description="Set the remote password other devices need to use the arm's bridge.")
    Group = Parser.add_mutually_exclusive_group()
    Group.add_argument("--generate", action="store_true", help="make a strong password, store it and print it once")
    Group.add_argument("--off", action="store_true", help="remove the password: only this PC can use the bridge")
    Group.add_argument("--status", action="store_true", help="say whether a password is set")
    Args = Parser.parse_args(Argv)
    File = RemoteAccess.PasswordPath()
    if Args.status:
        print(f"A remote password is set ({File})." if RemoteAccess.LoadPassword(File) is not None
              else "No remote password: only this PC can use the bridge.")
        return 0
    if Args.off:
        print("Remote password removed: only this PC can use the bridge now." if RemoteAccess.ClearPassword(File)
              else "No remote password was set: only this PC can use the bridge.")
        return 0
    if Args.generate:
        Password = Generate()
    else:
        Password = getpass.getpass("New remote password (at least 10 characters): ")
        Problem = RemoteAccess.ValidPasswordProblem(Password)
        if Problem:
            print(f"Not set: {Problem}.", file=sys.stderr)
            return 2
        if getpass.getpass("Again: ") != Password:
            print("Not set: the two did not match.", file=sys.stderr)
            return 2
    RemoteAccess.SetPassword(Password, File)
    if Args.generate:
        print(f"Remote password: {Password}")
        print("It is shown only now. Enter it once in Clancy on each other device.")
    print(f"Stored as a salted hash in {File}. The bridge and the dev server use it from their next request; "
          "devices that had the old one must enter this one.")
    return 0


if __name__ == "__main__":
    raise SystemExit(Main())
