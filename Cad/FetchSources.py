"""Fetch a pinned set of original SO-101 CAD and simulation files."""

import concurrent.futures as Futures
import hashlib as Hashlib
import json as Json
from pathlib import Path
import urllib.parse as UrlParse
import urllib.request as UrlRequest

ProjectRoot = Path(__file__).resolve().parents[1]
OutputRoot = Path.home() / "Downloads" / "SO101Cad"
Tree = Json.loads((ProjectRoot / "Cad" / "UpstreamTree.json").read_text(encoding="utf-8-sig"))
Commit = Tree["sha"]
Repository = "https://github.com/TheRobotStudio/SO-ARM100"


def SelectFile(SourcePath):
    if SourcePath in ("LICENSE", "README.md", "Simulation/README.md"):
        return True
    if SourcePath.startswith("STEP/SO101/") and "Leader_Specific" not in SourcePath:
        return SourcePath.endswith(".step")
    if SourcePath.startswith("Simulation/SO101/"):
        return True
    if SourcePath.startswith("STL/SO101/Individual/"):
        return SourcePath.rsplit("/", 1)[-1] not in (
            "Handle_SO101.stl", "Trigger_SO101.stl", "Wrist_Roll_SO101.stl", "SO101 Assembly.stl"
        )
    return False


def FetchFile(Entry):
    SourcePath = Entry["path"]
    Destination = OutputRoot / "Original" / SourcePath
    Destination.parent.mkdir(parents=True, exist_ok=True)
    SourceUrl = f"https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/{Commit}/{UrlParse.quote(SourcePath)}"
    if not Destination.exists():
        Request = UrlRequest.Request(SourceUrl, headers={"User-Agent": "SO101CadResearch"})
        with UrlRequest.urlopen(Request, timeout=90) as Response:
            Destination.write_bytes(Response.read())
    Data = Destination.read_bytes()
    BlobHash = Hashlib.sha1(f"blob {len(Data)}\0".encode() + Data).hexdigest()
    assert BlobHash == Entry["sha"], f"Upstream Git blob mismatch: {SourcePath}"
    return {
        "Path": str(Destination.relative_to(OutputRoot)).replace("\\", "/"),
        "SourceUrl": SourceUrl,
        "Bytes": len(Data),
        "GitBlobSha1": BlobHash,
        "Sha256": Hashlib.sha256(Data).hexdigest(),
    }


if __name__ == "__main__":
    Entries = [Entry for Entry in Tree["tree"] if Entry["type"] == "blob" and SelectFile(Entry["path"])]
    with Futures.ThreadPoolExecutor(max_workers=6) as Executor:
        Records = list(Executor.map(FetchFile, Entries))
    Manifest = {"Repository": Repository, "Commit": Commit, "Files": Records}
    (OutputRoot / "SourceManifest.json").write_text(Json.dumps(Manifest, indent=2), encoding="utf-8")
    print(f"Verified {len(Records)} original files against upstream Git blob hashes.")
    print(f"Pinned commit: {Commit}")
    print(OutputRoot)
