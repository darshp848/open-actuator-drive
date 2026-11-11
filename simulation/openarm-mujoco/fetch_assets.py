"""Acquire pinned official data assets; verify Git blobs and record provenance."""
import os
import concurrent.futures
import hashlib
import json
import shutil
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent
COMMIT = '06b4c8ad6d547c3faaa3bf780629ee3e1ac2b0d0'
REPO = 'https://github.com/enactic/openarm_mujoco'
TARGET = ROOT / 'vendor' / 'openarm_mujoco'
URDF = Path(os.environ.get('OPENARM_V2_URDF', ROOT / 'reference' / 'v2.urdf'))


def get(url):
    request = urllib.request.Request(url, headers={'User-Agent': 'OpenArm-research-setup'})
    with urllib.request.urlopen(request, timeout=45) as response:
        return response.read()


def main():
    tree = json.loads(get(f'https://api.github.com/repos/enactic/openarm_mujoco/git/trees/{COMMIT}?recursive=1'))
    assert not tree.get('truncated')
    items = [item for item in tree['tree'] if item['type'] == 'blob' and
             (item['path'].startswith('v2/') or item['path'] in ['README.md', 'LICENSE', 'pyproject.toml'])]
    TARGET.mkdir(parents=True, exist_ok=True)

    def fetch(item):
        destination = (TARGET / item['path']).resolve()
        assert destination.is_relative_to(TARGET.resolve())
        blob = destination.read_bytes() if destination.exists() else get(
            f'https://raw.githubusercontent.com/enactic/openarm_mujoco/{COMMIT}/{item["path"]}')
        git_hash = hashlib.sha1(f'blob {len(blob)}\0'.encode() + blob).hexdigest()
        assert git_hash == item['sha'], f'Source blob mismatch: {item["path"]}'
        if not destination.exists():
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(blob)
        return item['path'], hashlib.sha256(blob).hexdigest()

    with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
        hashes = dict(pool.map(fetch, items))
    reference = ROOT / 'reference'
    reference.mkdir(exist_ok=True)
    blob = URDF.read_bytes()
    assert hashlib.sha256(blob).hexdigest() == '045093bd5a08311d9ff81616b5504e643185f7d8872e33f924e552cde9ddb3cd'
    shutil.copyfile(URDF, reference / 'v2.urdf')
    provenance = dict(repository=REPO, commit=COMMIT, files_sha256=hashes,
                      reference_urdf_sha256=hashlib.sha256(blob).hexdigest(),
                      reference_urdf_commit='81fe64755f78777c9d55d7e5cc5a238a143f7950')
    (ROOT / 'assets-manifest.json').write_text(json.dumps(provenance, indent=2))
    print(f'Verified {len(hashes)} official files and reference URDF; no model compiled or simulated.')


if __name__ == '__main__':
    main()
