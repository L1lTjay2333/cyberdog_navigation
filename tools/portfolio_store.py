"""PC portfolio catalog and downloads."""
import json
import threading
import uuid
import zipfile
from pathlib import Path

from portfolio_imports import import_captures

_lock = threading.Lock()


def _write_manifest(manifest, works):
    temporary = manifest.with_suffix('.tmp')
    temporary.write_text(json.dumps(works, ensure_ascii=False, indent=2), encoding='utf-8')
    temporary.replace(manifest)


def _sync_catalog(root, folder):
    manifest = folder / 'manifest.json'
    works = json.loads(manifest.read_text(encoding='utf-8-sig'))
    removed = [json.loads(p.read_text(encoding='utf-8'))
               for p in (root/'evidence/portfolio_removed').glob('*.json')]
    added = list(import_captures(root, folder, works, removed))
    if added:
        works.extend(added)
        _write_manifest(manifest, works)
    return works


def portfolio_manifest(root):
    with _lock:
        return _sync_catalog(root, root/'assets/portfolio')


def portfolio_zip(root):
    folder = root/'assets/portfolio'
    with _lock:
        works = _sync_catalog(root, folder)
        payload = (folder/'manifest.json').read_bytes()
        target = folder/'portfolio.zip'
        manifest_name = 'CyberDog作品集/manifest.json'
        try:
            with zipfile.ZipFile(target) as saved:
                if saved.read(manifest_name) == payload:
                    return target.read_bytes()
        except (OSError, KeyError, zipfile.BadZipFile):
            pass
        names = {'index.html'}
        names.update(p.name for p in folder.iterdir() if p.suffix in {'.js', '.css'})
        for work in works:
            names.update(work['id'] + ext for ext in ('.png', '.bin', '.ply'))
            if work.get('map'):
                names.add(work['map'])
            for key in ('file', 'download'):
                if work.get('floor', {}).get(key):
                    names.add(work['floor'][key])
        temporary = folder / 'portfolio.next.zip'
        try:
            with zipfile.ZipFile(temporary, 'w', zipfile.ZIP_DEFLATED) as archive:
                archive.writestr(manifest_name, payload)
                for name in sorted(names):
                    path = (folder / name).resolve()
                    if path.parent != folder.resolve():
                        raise ValueError('作品资源路径无效')
                    if path.is_file():
                        archive.write(path, 'CyberDog作品集/' + name)
            temporary.replace(target)
        finally:
            temporary.unlink(missing_ok=True)
        return target.read_bytes()


def remove_work(root, work_id):
    if not isinstance(work_id, str) or not work_id:
        raise ValueError('请选择作品')
    folder = Path(root) / 'assets' / 'portfolio'
    with _lock:
        manifest = folder / 'manifest.json'
        works = _sync_catalog(Path(root), folder)
        removed = next((w for w in works if w['id'] == work_id), None)
        if removed is None:
            raise ValueError('作品已移除，请刷新页面')
        remaining = [w for w in works if w['id'] != work_id]
        # Retain the previous catalog for recovery; original and packaged data stay intact.
        backup = Path(root) / 'evidence' / 'portfolio_removed'
        backup.mkdir(parents=True, exist_ok=True)
        (backup / (uuid.uuid4().hex + '.json')).write_text(
            json.dumps(removed, ensure_ascii=False, indent=2), encoding='utf-8')
        _write_manifest(manifest, remaining)
        return {'ok': True, 'works': remaining, 'message': '已从作品集移除，原始采集数据仍保留'}
