"""Work around stock GetMapName selecting the first PGM, preserving old maps."""
import re
import shutil
import tempfile
from pathlib import Path


def verify_map_files(folder, name):
    folder=Path(folder).resolve()
    if not re.fullmatch(r'computer_room_[0-9]{8}_[0-9]{6}',name):
        raise RuntimeError('Invalid computer map name')
    for filename in (name+'.pgm',name+'.yaml',name+'.json','graph.pbstream','pose.txt'):
        candidate=folder/filename
        if candidate.is_symlink() or not candidate.is_file() or candidate.resolve().parent!=folder:
            raise RuntimeError('Incomplete/non-regular saved map: '+filename)
    return folder


def assert_active_map(folder,name):
    folder=verify_map_files(folder,name)
    if {p.name for p in folder.glob('*.pgm')}!={name+'.pgm'}:
        raise RuntimeError('Multiple stock maps can select wrong scene; archive old maps before localization')


def archive_other_maps(folder,name):
    folder=verify_map_files(folder,name)
    other=list(folder.glob('*.pgm'))
    other=[p for p in other if p.name!=name+'.pgm']
    targets=[]
    for pgm in other:
        for suffix in ('.pgm','.yaml','.json'):
            candidate=pgm.with_suffix(suffix)
            if not candidate.exists():continue
            if candidate.is_symlink() or not candidate.is_file() or candidate.resolve().parent!=folder:
                raise RuntimeError('Refuse non-regular/outside archive target')
            targets.append(candidate)
    if not targets:return {'backup':None,'archived':[],'map_name':name}
    result = finish_map_rotation(folder, name)
    return dict(backup=result['previous'], archived=[p.name for p in targets], map_name=name)


def stage_previous_map(active, emit):
    # Keep this snapshot across failed starts; replace it only after a successful save.
    if active.is_symlink() or not active.is_dir():
        raise RuntimeError('Invalid active map directory')
    if any(p.is_symlink() for p in active.rglob('*')):
        raise RuntimeError('Symlink in active map directory')
    pending = active.parent / 'computer_map_pending'
    if pending.is_symlink():
        raise RuntimeError('Invalid pending map directory')
    if pending.exists():
        if not (pending/'mapping'/'graph.pbstream').is_file():
            raise RuntimeError('Incomplete previous map snapshot')
        return
    if not list(active.glob('*.pgm')):
        return
    if not (active/'graph.pbstream').is_file():
        raise RuntimeError('Incomplete active map')
    previous = active.parent/'computer_map_previous'
    if previous.is_symlink() or (previous.exists() and (not (previous/'mapping'/'graph.pbstream').is_file()
            or any(p.is_symlink() for p in previous.rglob('*')))):
        raise RuntimeError('Invalid previous map directory')
    tmp = Path(tempfile.mkdtemp(prefix='computer_map_stage_', dir=str(active.parent)))
    try:
        shutil.copytree(str(active), str(tmp/'mapping'))
        tmp.rename(pending)
    finally:
        if tmp.exists():
            shutil.rmtree(str(tmp))
    emit('BACKUP', path=str(pending/'mapping'))


def finish_map_rotation(active, name):
    verify_map_files(active, name)
    pending = active.parent/'computer_map_pending'
    previous = active.parent/'computer_map_previous'
    for slot in (pending, previous):
        if slot.is_symlink() or (slot.exists() and (any(p.is_symlink() for p in slot.rglob('*')) or not (slot/'mapping'/'graph.pbstream').is_file())):
            raise RuntimeError('Invalid map rotation slot')
    others = [p for p in active.glob('*.pgm') if p.stem != name]
    if others and not pending.exists():
        raise RuntimeError('Previous snapshot missing; preserving existing maps')
    for pgm in others:
        for suffix in ('.pgm','.yaml','.json'):
            path = pgm.with_suffix(suffix)
            if path.is_symlink():
                raise RuntimeError('Unexpected map symlink')
            if path.is_file():
                path.unlink()
    assert_active_map(active, name)
    if pending.exists():
        if previous.exists():
            shutil.rmtree(str(previous))
        pending.rename(previous)
    return dict(map_name=name, previous=str(previous) if previous.exists() else None)
