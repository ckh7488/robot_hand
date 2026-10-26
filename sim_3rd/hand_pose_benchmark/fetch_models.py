"""Download pinned public inference assets and retain provenance (no credentials)."""
import concurrent.futures
import hashlib
import json
import pathlib
import time

import requests

ROOT = pathlib.Path(r'E:\hand_recog\vision-runtime\models')
SOURCE = pathlib.Path(__file__).resolve().parents[2] / 'docs/reviews/VISION001/local/sources.json'


def download(url, target, sha=None, size=None):
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists() and (size is None or target.stat().st_size == size):
        with target.open('rb') as f:
            actual = hashlib.file_digest(f, 'sha256').hexdigest()
        if sha is None or actual == sha:
            return {'path': str(target), 'url': url, 'bytes': target.stat().st_size, 'sha256': actual}
    session = requests.Session()
    session.trust_env = False
    temp = target.with_suffix(target.suffix + '.part')
    for attempt in range(4):
        try:
            done = temp.stat().st_size if temp.exists() else 0
            headers = {'Range': f'bytes={done}-'} if done else {}
            with session.get(url, headers=headers, stream=True, timeout=(30, 120)) as response:
                response.raise_for_status()
                append = done and response.status_code == 206
                if not append:
                    done = 0
                with temp.open('ab' if append else 'wb') as f:
                    for chunk in response.iter_content(8 * 1024 * 1024):
                        f.write(chunk)
            if size is not None and temp.stat().st_size != size:
                raise ValueError('Downloaded size mismatch: ' + target.name)
            with temp.open('rb') as f:
                actual = hashlib.file_digest(f, 'sha256').hexdigest()
            if sha is not None and actual != sha:
                raise ValueError('Downloaded SHA256 mismatch: ' + target.name)
            temp.replace(target)
            print('DOWNLOADED', target.name, target.stat().st_size, flush=True)
            return {'path': str(target), 'url': url, 'bytes': target.stat().st_size, 'sha256': actual}
        except (requests.RequestException, OSError):
            if attempt == 3:
                raise
            time.sleep(2)


def main():
    sources = json.loads(SOURCE.read_text(encoding='utf8'))
    jobs = [(f['source'], ROOT / 'wilor' / f['file'], f['sha256'], f['size'])
            for f in sources['weights']['files']]
    # Author-owned public demo mirror. The 6GB university archive stalls here;
    # download only the unchanged HaMeR checkpoint and its config instead.
    prefix = 'https://huggingface.co/spaces/geopavlakos/HaMeR/resolve/aabc78ee4fd1f1ccf1a55fdd4225bf84df0e680a/'
    jobs.extend([
        (prefix+'_DATA/hamer_ckpts/checkpoints/hamer.ckpt',ROOT/'hamer/_DATA/hamer_ckpts/checkpoints/hamer.ckpt',
         'e5cc06f294d88a92dee24e603480aab04de532b49f0e08200804ee7d90e16f53',2689536166),
        (prefix+'_DATA/hamer_ckpts/model_config.yaml',ROOT/'hamer/_DATA/hamer_ckpts/model_config.yaml',None,None),
        (prefix+'_DATA/hamer_ckpts/dataset_config.yaml',ROOT/'hamer/_DATA/hamer_ckpts/dataset_config.yaml',None,None),
    ])
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
        receipts = list(pool.map(lambda args: download(*args), jobs))
    (ROOT / 'provenance.json').write_text(json.dumps(receipts, indent=2), encoding='utf8')
    print('MODELS_READY', flush=True)


if __name__ == '__main__':
    main()
