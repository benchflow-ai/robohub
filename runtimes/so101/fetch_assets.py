"""Build-time download of pinned robot model assets (MuJoCo Menagerie meshes) for a Robo Use runtime.

    python fetch_assets.py <dest> <manifest.json>...

Each manifest lists files with a URL at a pinned upstream commit and a SHA-256. Every file is checked; a
mismatch fails the image build. Standard library only.
"""
import hashlib
import json
import sys
import time
import urllib.request
from pathlib import Path


def main() -> int:
    dest = Path(sys.argv[1])
    n = 0
    for manifest in sys.argv[2:]:
        for f in json.loads(Path(manifest).read_text())["files"]:
            out = dest / f["path"]
            out.parent.mkdir(parents=True, exist_ok=True)
            for attempt in range(4):
                try:
                    with urllib.request.urlopen(f["url"], timeout=120) as r:
                        data = r.read()
                    break
                except OSError:
                    if attempt == 3:
                        raise
                    time.sleep(2 ** attempt)
            got = hashlib.sha256(data).hexdigest()
            if got != f["sha256"]:
                sys.exit(f"sha256 mismatch for {f['url']}: got {got}, pinned {f['sha256']}")
            out.write_bytes(data)
            n += 1
    print(f"{n} asset file(s) fetched and verified into {dest}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
