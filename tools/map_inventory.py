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
    # Full backup finishes BEFORE any file is moved. Backup remains outside
    # the active folder so the stock service cannot select its old PGM files.
    backup=Path(tempfile.mkdtemp(prefix='computer_active_map_backup_',dir=str(folder.parent)))
    shutil.copytree(str(folder),str(backup/'mapping'))
    archive=backup/'old_maps';archive.mkdir()
    for candidate in targets:shutil.move(str(candidate),str(archive/candidate.name))
    assert_active_map(folder,name)
    return {'backup':str(backup),'archived':[p.name for p in targets],'map_name':name}
