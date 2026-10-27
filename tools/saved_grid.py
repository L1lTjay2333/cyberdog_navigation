"""Read stock P5/YAML saved map for display, never authorize navigation."""
import json
import math
import re
import time
from pathlib import Path

def load_saved_grid(directory,name):
    if not isinstance(name,str) or not re.fullmatch(r'[A-Za-z0-9_-]+',name):
        raise ValueError('Invalid saved map name')
    root=Path(directory);meta={}
    for line in (root/(name+'.yaml')).read_text().splitlines():
        if ':' in line:
            key,value=line.split(':',1);meta[key.strip()]=value.strip()
    image=meta['image'].strip('\"\'')
    if Path(image).name!=name+'.pgm':raise ValueError('Map image identity mismatch')
    raw=(root/(name+'.pgm')).read_bytes();position=0;tokens=[]
    while len(tokens)<4:
        while position<len(raw) and raw[position] in b' \r\n\t':position+=1
        if position>=len(raw):raise ValueError('Truncated PGM')
        if raw[position]==35:
            position=raw.index(b'\n',position)+1;continue
        start=position
        while position<len(raw) and raw[position] not in b' \r\n\t':position+=1
        tokens.append(raw[start:position])
    if tokens[0]!=b'P5' or tokens[3]!=b'255':raise ValueError('Unsupported PGM')
    width,height=int(tokens[1]),int(tokens[2])
    position+=2 if raw[position:position+2]==b'\r\n' else 1
    pixels=raw[position:]
    if min(width,height)<=0 or len(pixels)!=width*height:raise ValueError('Incomplete PGM pixels')
    resolution=float(meta['resolution']);origin=json.loads(meta['origin'])
    if len(origin)!=3 or resolution<=0 or not all(math.isfinite(v) for v in origin+[resolution]):raise ValueError('Invalid map geometry')
    free,occupied=float(meta['free_thresh']),float(meta['occupied_thresh'])
    if not 0<=free<occupied<=1:raise ValueError('Invalid map thresholds')
    negate=int(meta.get('negate','0'))
    if negate not in (0,1):raise ValueError('Invalid negate')
    table=[]
    for value in range(256):
        probability=(value if negate else 255-value)/255.
        table.append(100 if probability>occupied else 0 if probability<free else -1)
    data=[]
    for row in range(height-1,-1,-1):data.extend(table[v] for v in pixels[row*width:(row+1)*width])
    return dict(width=width,height=height,resolution=resolution,frame='map',
        origin=dict(x=origin[0],y=origin[1],qz=math.sin(origin[2]/2),qw=math.cos(origin[2]/2)),
        data=data,received_unix=time.time(),display_only=True,source='saved_file',map_name=name)
