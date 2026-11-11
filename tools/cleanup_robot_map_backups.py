"""Remove legacy robot map backups, retaining current and one distinct predecessor."""
import hashlib
import json
import shutil
from pathlib import Path

base = Path('/home/mi')
active = base/'mapping'
def checked_tree(path):
    if path.is_symlink() or path.resolve().parent != base or not path.is_dir():
        raise RuntimeError('Unsafe map directory: '+str(path))
    if any(p.is_symlink() for p in path.rglob('*')):
        raise RuntimeError('Symlink inside map directory: '+str(path))
def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()
checked_tree(active)
current = digest(active/'graph.pbstream')
candidates = sorted(
    [p for p in base.iterdir() if p.name.startswith(
        ('computer_map_backup_', 'computer_active_map_backup_', 'computer_depth_rollback_backup_'))],
    key=lambda p:p.stat().st_mtime, reverse=True)
for path in candidates:
    checked_tree(path)
previous = base/'computer_map_previous'
if previous.exists():
    checked_tree(previous)
    keep = previous
else:
    keep = next((p for p in candidates if (p/'mapping/graph.pbstream').is_file()
                 and digest(p/'mapping/graph.pbstream') != current), None)
if keep is None:
    raise RuntimeError('No distinct previous map verified')
if not list((keep/'mapping').glob('*.pgm')) or not (keep/'mapping/pose.txt').is_file():
    raise RuntimeError('Previous map incomplete')
removed=[]
for path in candidates:
    if path == keep:
        continue
    shutil.rmtree(str(path))
    removed.append(path.name)
if keep != previous:
    keep.rename(previous)
print(json.dumps(dict(active=str(active),previous=str(previous),removed=removed,
                     free_bytes=shutil.disk_usage(str(base)).free)))
