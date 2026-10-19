"""Build the public, code-only Clancy snapshot as a commit on the local branch public-main. It never pushes.

  python PublishPublic.py                      # HEAD minus PublicExclude.txt, onto public-main
  python PublishPublic.py --public-files       # the same, plus the public-only LICENSE, NOTICE and README note
  python PublishPublic.py --commit public-prep --public-files
  python PublishPublic.py --dry-run            # list what would be published and left out; commit nothing

The tree is the given commit's files minus every path PublicExclude.txt matches (read from that commit). It is built in
a temporary index (GIT_INDEX_FILE), so the working tree, the real index and every other branch stay as they are. The
commit's author and committer are the GitHub noreply identity below. When public-main already exists its tip is the
parent, so each publish stacks as an ordinary commit; a tree identical to the tip's makes no commit.

--public-files adds what only the public repository has: LICENSE (the Apache-2.0 text of AI Controlled
WebApp/public/Cad/LICENSE.txt), NOTICE (copyright and third-party attributions) and, in README.md, a short note that
the repository is code only, with image links to left-out files dropped. Use it for every real publish: a publish
without it leaves those files out of that commit.
"""
from __future__ import annotations

import argparse as Argparse
import fnmatch as Fnmatch
import os as Os
import re as Re
import subprocess as Subprocess
import sys as Sys
import tempfile as Tempfile
import urllib.parse as UrlParse
from pathlib import Path

Repo = Path(__file__).resolve().parent
Branch = "public-main"
AuthorName = "Aaran"
AuthorEmail = "164961085+Aaran-Chen@users.noreply.github.com"
ExcludeFile = "PublicExclude.txt"
LicenseSource = "AI Controlled WebApp/public/Cad/LICENSE.txt"
SoArmSource = "AI Controlled WebApp/public/Cad/Source.txt"
ZeroSha = "0" * 40

PublicNote = """## About this public repository

Clancy is a whole XLeRobot: an SO-101 arm on a cart with an Orbbec depth camera, driven by a local Ollama vision model
through the web app and the Python bridge below. This repository is code only. Images, recorded and measured data
(camera calibration, the saved home pose, servo backups, recorded poses and replay fixtures) and the generated meshes
are left out, so a fresh clone needs these before the app builds and the bridge starts:

1. The meshes and collision surfaces, rebuilt by the scripts in `Cad/` (see `Cad/ReadMe.md`), from the repository root:
   `pip install -r Cad/Requirements.txt`, then `python Cad/FetchSources.py` (downloads the pinned SO-ARM100 files to
   `~/Downloads/SO101Cad`), `python Cad/PrepareGuiAssets.py` (`SO101Parts.glb`), a clone of
   [Vector-Wangel/XLeRobot](https://github.com/Vector-Wangel/XLeRobot) at `../XLeRobot-main` and
   `python Cad/PrepareXLeRobotAssets.py` (`XLeRobotBody.glb`), and last `python Cad/PrepareCollisionModel.py`
   (`back end/BasketSurface.json` and `back end/RobotBodySurface.json`).
2. `back end/home_preset.json`, the saved home pose in raw servo steps, which the app's build reads. Placeholders
   let it build: `{"shoulder_pan": 2048, "shoulder_lift": 2048, "elbow_flex": 2048, "wrist_flex": 2048, "wrist_roll": 2048, "gripper": 2048}`.
   They are not a pose for your arm: put the arm in its home pose and press **Set Home** (which rewrites the file)
   before you ever press **Home**.
3. Your own camera calibration (`back end/camera_calibration.json`), made with the app's calibration.

The replay tests that read recorded frames (`back end/TestFixtures/`, `back end/captures/`) report them missing.

"""


def Git(*Arguments, Input=None, Env=None, Check=True):
    Result = Subprocess.run(["git", *Arguments], cwd=Repo, input=Input, capture_output=True, env=Env)
    if Check and Result.returncode != 0:
        raise SystemExit(f"git {' '.join(Arguments)} failed: {Result.stderr.decode(errors='replace').strip()}")
    return Result


def ParsePatterns(Text):
    Patterns = []
    for Line in Text.splitlines():
        Line = Line.split(" # ", 1)[0].strip()
        if Line and not Line.startswith("#"):
            Patterns.append(Line)
    return Patterns


def IsExcluded(PathName, Patterns):
    Lower = PathName.lower()
    for Pattern in Patterns:
        Pattern = Pattern.lower()
        if Pattern.endswith("/"):
            if Lower.startswith(Pattern) or Fnmatch.fnmatchcase(Lower, Pattern + "*"):
                return True
        elif Fnmatch.fnmatchcase(Lower, Pattern):
            return True
    return False


def TreeEntries(Commit):
    # (mode, type, sha, path) for every file of the commit, paths exactly as git stores them.
    Raw = Git("ls-tree", "-r", "-z", "--full-tree", Commit).stdout.decode("utf-8")
    Entries = []
    for Record in Raw.split("\0"):
        if Record:
            Meta, PathName = Record.split("\t", 1)
            Mode, Kind, Sha = Meta.split(" ")
            Entries.append((Mode, Kind, Sha, PathName))
    return Entries


def ReadBlob(Commit, PathName):
    return Git("show", f"{Commit}:{PathName}").stdout


def StoreBlob(Data):
    return Git("hash-object", "-w", "--stdin", Input=Data).stdout.decode().strip()


def NoticeText(Commit):
    Source = ReadBlob(Commit, SoArmSource).decode("utf-8")
    Match = Re.search(r"^Commit:\s*([0-9a-f]{40})", Source, Re.M)
    if not Match:
        raise SystemExit(f"No source commit in {SoArmSource}")
    return f"""Clancy
Copyright 2026 Aaran Chen

Licensed under the Apache License, Version 2.0 (see LICENSE).

This product includes or derives from the following third-party work:

SO-ARM100 (SO-101 follower CAD, URDF and joint geometry)
  by TheRobotStudio and contributors, https://github.com/TheRobotStudio/SO-ARM100
  source commit {Match.group(1)}
  Apache License 2.0 (AI Controlled WebApp/public/Cad/LICENSE.txt)

XLeRobot (cart, mast and camera head geometry)
  by Vector-Wangel and contributors, https://github.com/Vector-Wangel/XLeRobot
  Apache License 2.0 (AI Controlled WebApp/public/Cad/XLeRobotLicense.txt)

Play font
  by Jonas Hecksher, Playtypes, e-types AS, installed from npm (@fontsource/play)
  SIL Open Font License 1.1 (AI Controlled WebApp/public/Fonts/Play-LICENSE.txt)

back end/feetech_simple.py follows the Feetech motor-bus conventions of Hugging Face LeRobot
  (https://github.com/huggingface/lerobot), Apache License 2.0.
"""


def PublicReadme(Text, Patterns):
    Newline = "\r\n" if "\r\n" in Text else "\n"
    Lines = Text.replace("\r\n", "\n").split("\n")
    Kept = []
    for Line in Lines:
        Image = Re.fullmatch(r"\s*!\[[^\]]*\]\(([^)\s]+)[^)]*\)\s*", Line)
        if Image and IsExcluded(UrlParse.unquote(Image.group(1)), Patterns):
            continue  # an image the public repo does not have
        Kept.append(Line)
    # The note goes before the first horizontal rule, right after the introduction.
    Index = next((Number for Number, Line in enumerate(Kept) if Line.strip() == "---"), 1)
    Start = Index
    while Start > 0 and Kept[Start - 1].strip() == "":
        Start -= 1
    Note = PublicNote.strip("\n").split("\n")
    Kept[Start:Index] = [""] + Note + [""]
    return Newline.join(Kept)


def Main():
    Parser = Argparse.ArgumentParser(description=__doc__, formatter_class=Argparse.RawDescriptionHelpFormatter)
    Parser.add_argument("--commit", default="HEAD", help="the private commit to publish (default HEAD)")
    Parser.add_argument("--public-files", action="store_true", help="also write LICENSE, NOTICE and the README note")
    Parser.add_argument("--dry-run", action="store_true", help="list the published and left-out paths, commit nothing")
    Parser.add_argument("--message", help="the commit message (default names the private commit)")
    Arguments = Parser.parse_args()

    Commit = Git("rev-parse", "--verify", f"{Arguments.commit}^{{commit}}").stdout.decode().strip()
    Patterns = ParsePatterns(ReadBlob(Commit, ExcludeFile).decode("utf-8"))
    Entries = TreeEntries(Commit)
    Published = [Entry for Entry in Entries if not IsExcluded(Entry[3], Patterns)]
    LeftOut = [Entry for Entry in Entries if IsExcluded(Entry[3], Patterns)]
    Unsupported = [Entry[3] for Entry in Published if Entry[1] != "blob"]
    if Unsupported:
        raise SystemExit(f"Not plain files (submodules?): {Unsupported}")

    Extra = {}
    if Arguments.public_files:
        Extra["LICENSE"] = ReadBlob(Commit, LicenseSource)
        Extra["NOTICE"] = NoticeText(Commit).encode("utf-8")
        Extra["README.md"] = PublicReadme(ReadBlob(Commit, "README.md").decode("utf-8"), Patterns).encode("utf-8")

    if Arguments.dry_run:
        for Entry in LeftOut:
            print("left out ", Entry[3])
        for Name in Extra:
            print("public   ", Name)
        print(f"{len(Published)} files published from {Commit[:12]}, {len(LeftOut)} left out, {len(Extra)} public-only")
        return

    Rows = {Entry[3]: (Entry[0], Entry[2]) for Entry in Published}
    for Name, Data in Extra.items():
        Rows[Name] = ("100644", StoreBlob(Data))
    IndexInfo = b"".join(f"{Mode} {Sha}\t{Name}".encode("utf-8") + b"\0" for Name, (Mode, Sha) in sorted(Rows.items()))
    with Tempfile.TemporaryDirectory() as Folder:
        Env = {Key: Value for Key, Value in Os.environ.items() if not Key.startswith("GIT_")}
        Env["GIT_INDEX_FILE"] = str(Path(Folder) / "index")
        Git("update-index", "-z", "--index-info", Input=IndexInfo, Env=Env)
        Tree = Git("write-tree", Env=Env).stdout.decode().strip()

    Parent = Git("rev-parse", "--verify", "-q", f"refs/heads/{Branch}", Check=False).stdout.decode().strip()
    if Parent and Git("rev-parse", f"{Parent}^{{tree}}").stdout.decode().strip() == Tree:
        print(f"{Branch} already holds this tree at {Parent}; nothing committed.")
        return
    Message = Arguments.message or f"Clancy public snapshot (code only) from private commit {Commit[:12]}"
    Env = dict(Os.environ, GIT_AUTHOR_NAME=AuthorName, GIT_AUTHOR_EMAIL=AuthorEmail,
               GIT_COMMITTER_NAME=AuthorName, GIT_COMMITTER_EMAIL=AuthorEmail)
    ParentArguments = ["-p", Parent] if Parent else []
    New = Git("commit-tree", Tree, *ParentArguments, Input=(Message + "\n").encode("utf-8"), Env=Env).stdout.decode().strip()
    Git("update-ref", "-m", "PublishPublic", f"refs/heads/{Branch}", New, Parent or ZeroSha)
    print(f"{Branch} -> {New}")
    print(f"  from {Commit} ({len(Published)} files published, {len(LeftOut)} left out, {len(Extra)} public-only)")
    print(f"  parent {Parent or '(none: first commit)'}")
    print(f"  tree {Tree}")
    print("Not pushed. Inspect it with: git log --stat " + Branch)


if __name__ == "__main__":
    Sys.exit(Main())
