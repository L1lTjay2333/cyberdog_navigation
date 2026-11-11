import React, {useState} from 'react';
import {createRoot} from 'react-dom/client';
import FlipCard from './FlipCard';
import './portfolio.css';

function Card({work, remove, select}) {
  const [flipped, flip] = useState(false);
  const [confirming, confirm] = useState(false);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState('');
  async function erase() {
    setBusy(true); setError('');
    try {
      const r = await fetch('/portfolio/delete', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({id:work.id})});
      const result = await r.json();
      if (!r.ok || !result.ok) throw Error(result.error || result.message || '删除失败');
      remove(result.works);
    } catch(e) {setError(e.message); setBusy(false);}
  }
  const stop=e=>e.stopPropagation();
  const label = work.kind === 'map' ? '查看地图' : work.kind === 'raw' ? '查看采集记录' : '查看点云';
  const details = work.kind === 'map' ? '二维地图' : work.kind === 'raw' ? '原始帧已归档' : `${work.points.toLocaleString()} 个点 / 顶点`;
  return <article data-work-id={work.id}>
    <FlipCard width={300} height={270} background="#fffcf4" color="#39352f" shadowOpacity={0.12}
      ariaLabel={work.title+'，点击翻面'} flipped={flipped} onFlipChange={v=>{flip(v);confirm(false)}}
      front={<div className="portfolio-front"><img src={'./'+work.id+'.png'} alt={work.title+'预览'}/><h3>{work.title}</h3><p>{work.date} · 左键点击翻面</p></div>}
      back={<div className="portfolio-back"><h3>{work.title}</h3><p>{details}</p>
        <div className="portfolio-actions" onPointerDown={stop} onPointerUp={stop} onClick={stop} onKeyDown={stop}>
          {confirming ? <><p>移除这件作品？原始采集数据保留。</p><button disabled={busy} className="danger" onClick={erase}>{busy?'正在移除…':'确认删除'}</button><button disabled={busy} onClick={()=>confirm(false)}>取消</button></> : <><button onClick={()=>select(work)}>{label}</button><button className="danger" onClick={()=>confirm(true)}>删除此作品</button></>}
          <p role="status">{error}</p>
        </div></div>}/>
  </article>;
}
window.mountPortfolioCards = (works, select, changed) => {
  function Cards() {
    const [items, setItems] = useState(works);
    const remove = next => {setItems(next);changed(next)};
    return items.length ? items.map(work=><Card key={work.id} work={work} select={select} remove={remove}/>) : <p>作品集为空，新采集成果可继续加入。</p>;
  }
  createRoot(document.getElementById('cards')).render(<Cards/>);
};
