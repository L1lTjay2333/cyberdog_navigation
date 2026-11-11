import './PanelGlass';
import React from 'react';
import { createRoot } from 'react-dom/client';
import DecryptedText from './DecryptedText/DecryptedText';

const navigationTitles = [
  '.brand',
  '#workflow-bar > strong',
  '#navigation-actions > strong',
  '#manual-panel > h2',
  '#live-depth-panel > summary',
  '#map-panel > h2',
  '#scene > h2'
];
const modelTitles = ['main > h1', '#scan-remote > h2'];
const selectors = document.getElementById('navigation-background') ? navigationTitles : modelTitles;

for (const selector of selectors) {
  const title = document.querySelector(selector);
  if (!title) continue;
  const text = title.textContent.trim();
  const host = document.createElement('span');
  host.className = 'decrypted-heading';
  title.replaceChildren(host);
  createRoot(host).render(
    <DecryptedText
      text={text}
      animateOn="inViewHover"
      effect="scramble"
      sequential={true}
      revealDirection="start"
      speed={60}
      maxIterations={10}
      hoverRadius={100}
      scrambleColor="#2bb673"
      loop={false}
      loopDelay={2000}
      useOriginalCharsOnly={false}
      characters="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz!@#$%^&*()_+"
      className="revealed"
      encryptedClassName="encrypted"
    />
  );
}
