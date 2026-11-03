"""Build a self-contained offline UI for archived RGB-depth diagnostic images.

Uses local relative image links; no robot connection, server or capture.
"""
import json
from pathlib import Path
import os
import base64
import copy

ROOT=Path(__file__).resolve().parents[1]


def build():
    output=ROOT/'evidence/rgbd_review_20261030'
    output.mkdir(exist_ok=True)
    sessions=[('20261030_171129_366554','椅子'),('20261030_172452_014781','大隔板'),
        ('20261030_173035_708434','地面箱体'),('20261030_173413_005969','首次垫高'),
        ('20261030_173707_113882','再次垫高')]
    rows=[]
    def link(path):
        if not path.is_file():raise FileNotFoundError(path)
        return Path(os.path.relpath(path,output)).as_posix()
    for session,label in sessions:
        folder=ROOT/'evidence/rgbd_samples'/session
        preview=folder/'projection_foreground_check'
        if not (preview/'report.json').exists():continue
        report=json.loads((preview/'report.json').read_text(encoding='utf-8'))
        for pair in report['pairs']:
            i=pair['pair_id']
            rows.append({'label':label+' · 第%d对'%(i+1),'session':session,
                'difference_ms':round(pair['time_difference_s']*1000,2),
                'original':link(preview/('pair_%s_original.png'%i)),
                'depth':link(preview/('pair_%s_depth_boundary_types.png'%i)),
                'variants':{name:link(preview/info['preview']) for name,info in pair['variants'].items()},
                'edges':pair['depth_edge_points'],
                'holes':pair['invalid_neighbor_boundary_points']})
    if not rows:raise ValueError('No diagnostic frames')
    # Keep the manifest readable, but embed images into the delivered HTML.
    # App previews may copy the HTML without its sibling image directories.
    embedded=copy.deepcopy(rows)
    cache={}
    def embed(name):
        if name not in cache:
            cache[name]='data:image/png;base64,'+base64.b64encode((output/name).read_bytes()).decode('ascii')
        return cache[name]
    for row in embedded:
        for key in ('original','depth'):row[key]=embed(row[key])
        row['variants']={key:embed(value) for key,value in row['variants'].items()}
    initial=embedded[-1]
    data=json.dumps(embedded,ensure_ascii=False).replace('<','\\u003c')
    html='''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RGB与深度离线对照</title>
<style>body{margin:24px;background:#101923;color:#e5edf6;font:16px/1.6 system-ui}main{max-width:1400px;margin:auto}h1{font-size:26px}select,button{padding:10px;background:#23364b;color:white;border:1px solid #678;border-radius:6px}label{display:inline-block;margin:8px 20px 8px 0}.views{display:grid;grid-template-columns:1fr 1fr;gap:18px}.stack{position:relative;aspect-ratio:4/3;background:#000}.stack img{position:absolute;width:100%;height:100%;object-fit:contain}figure{margin:0}figure>img{width:100%;display:block}figcaption,.info{color:#adc6dd}.status{padding:12px;border-left:4px solid #efb45c;background:#25303b}input{vertical-align:middle}@media(max-width:850px){.views{grid-template-columns:1fr}}</style>
<main><h1>RGB与深度离线对照</h1>
<p><a style="color:#8dd5ff" href="static_consensus.html">查看同一静止批次的原始深度与多数一致处理结果</a></p>
<p class="status">空间对齐尚未验证。此页面只查看已保存数据，不连接机器人。不同矩阵解释仅供排错，不能按观感自动选为真实标定。</p>
<label>记录 <select id="record"></select></label>
<label>投影解释 <select id="variant"><option value="source_array_column_major">米制与Brown畸变假设</option><option value="native_ptx_public_call_assumption">真机PTX＋公开调用假设（待输出核对）</option><option value="mathematical_inverse">普通数学逆矩阵（诊断）</option><option value="direct_matrix">直接矩阵（诊断）</option></select></label>
<label>绿色轮廓显示强度 <input id="opacity" type="range" min="0" max="100" value="100"></label>
<p id="metadata" class="info"></p>
<div class="views"><figure><div class="stack"><img id="original" alt="原始彩色照片"><img id="overlay" alt="投影后的有效深度前景边界"></div><figcaption>彩色相机画面。绿色是两个有效深度之间跳变的较近侧，仍可能受跨相机遮挡影响。</figcaption></figure>
<figure><img id="depth" alt="深度相机自身坐标中的边界分类"><figcaption>深度相机自身画面，未对齐至彩色图。绿色：有效深度跳变的前景侧；橙色：有效深度邻接无效或超范围深度；黑色：无有效深度。橙色不能直接视为物体真实轮廓。</figcaption></figure></div>
<p>投影已按目标像素最近深度排除部分遮挡边界，容差2厘米。PTX分支依据已保存的真机内核以及公开调用的缩放1、模型5，显示中心点近似；运行时参数与原厂像素角点栅格化尚未交叉验证。所有分支均未通过空间配准验收。</p>
<p>时间差是消息时间戳差，并非已确认的曝光同步误差。画面不完整、黑色表面或遮挡均可能使轮廓不足；本页不据此判断测距精度或自动更改标定。</p></main>
<script>const rows=DATA;const el=id=>document.getElementById(id);rows.forEach((r,i)=>el('record').add(new Option(r.label,i)));el('record').value=rows.length-1;
function show(){const r=rows[Number(el('record').value)];el('original').src=r.original;el('overlay').src=r.variants[el('variant').value];el('depth').src=r.depth;el('overlay').style.opacity=el('opacity').value/100;el('metadata').textContent='记录 '+r.session+' ｜消息时间差 '+r.difference_ms+' 毫秒 ｜前景边界 '+r.edges+' 点 ｜无效邻域边界 '+r.holes+' 点';}
['record','variant','opacity'].forEach(id=>el(id).addEventListener('input',show));show();</script></html>'''.replace('const rows=DATA;','const rows='+data+';')
    # Initial pictures work even when a preview disables JavaScript.
    html=html.replace('id="original" alt=', 'id="original" src="'+initial['original']+'" alt=')
    html=html.replace('id="overlay" alt=', 'id="overlay" src="'+initial['variants']['source_array_column_major']+'" alt=')
    html=html.replace('id="depth" alt=', 'id="depth" src="'+initial['depth']+'" alt=')
    html=html.replace('<p id="metadata" class="info"></p>',
        '<p id="metadata" class="info">'+initial['label']+'</p><noscript>当前显示最后一次记录。请在普通浏览器中打开以切换其他记录。</noscript>')
    (output/'index.html').write_text(html,encoding='utf-8')
    (output/'manifest.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'page':str(output/'index.html'),'pairs':len(rows),'robot_connections':0},ensure_ascii=False))


if __name__=='__main__':build()
