import './PanelGlass.css';

const navigation = document.getElementById('navigation-background');
const selector = navigation
  ? '#workflow-bar, #manual-panel, #end-task-panel, main>section>.card, .sensor-panel, .metric'
  : '#model-toolbar, .scan-main';

for (const panel of document.querySelectorAll(selector)) {
  if (getComputedStyle(panel).position === 'static') panel.style.setProperty('position', 'relative', 'important');
  panel.classList.add('react-glass-panel');
  panel.style.setProperty('background', 'linear-gradient(135deg,rgba(255,252,243,.38),rgba(244,236,219,.22))', 'important');
  panel.style.setProperty('backdrop-filter', 'none', 'important');
  panel.style.setProperty('-webkit-backdrop-filter', 'none', 'important');
  const host = document.createElement('div');
  host.className = 'panel-glass-layer';
  host.setAttribute('aria-hidden', 'true');
  panel.append(host);
  const radius = parseFloat(getComputedStyle(panel).borderRadius) || 12;
  // Acquisition pages use a lightweight glass layer; full-screen SVG refraction
  // competes with point-cloud rendering and remote-control input.
  host.classList.add('panel-glass-lightweight');
}
