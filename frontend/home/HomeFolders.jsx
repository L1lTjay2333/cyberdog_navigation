import React from 'react';
import { createRoot } from 'react-dom/client';
import Folder from './Folder';
import './HomeFolders.css';

const items = [
  { title: '3D 建模', href: '/model/' },
  { title: '地图扫描', href: '/navigation' },
  { title: '作品集', href: '/portfolio/' }
];

createRoot(document.getElementById('home-folders')).render(
  <div className="folder-option" data-module="workspace">
    <div className="folder-surface">
      <Folder
        label="功能工作台"
        items={items}
        variant="glass"
        color="#7aa7ff"
        theme="light"
        size={1.05}
        draggable={false}
        zoomOnClick={false}
        onItemClick={item => window.location.assign(item.href)}
      />
    </div>
  </div>
);
