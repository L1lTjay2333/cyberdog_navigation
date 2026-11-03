"""Repeat bounded local alignment checks on archived data; no robot access."""
import argparse
import hashlib
import json
from pathlib import Path
from reconstruction_turn_audit import load_views
from reconstruction_local import joint_align


def audit(session,output,starts):
    output=Path(output)
    if output.exists():raise FileExistsError('Use a new report file')
    frames,clouds,_,sha=load_views(session,max_depth=2.)
    if not 1<=len(starts)<=12 or any(type(i) is not int or i<1 or i+9>len(frames) for i in starts):
        raise ValueError('Expected up to 12 valid ten-frame windows')
    rows=[]
    for first in starts:
        _,report=joint_align(clouds[first-1:first+9])
        rows.append(dict(first_frame=first,**report))
        print(first,report['accepted'],report['reason'],flush=True)
    if hashlib.sha256((Path(session)/'frames.jsonl').read_bytes()).hexdigest()!=sha:raise ValueError('Source changed')
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps({'source_sha256':sha,'windows':rows,'source_unchanged':True,
        'absolute_accuracy_validated':False},indent=2),encoding='utf-8')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('session');p.add_argument('output')
    p.add_argument('--starts',type=int,nargs='+',default=[1,11,41,131,231,321])
    args=p.parse_args();audit(args.session,args.output,args.starts)
