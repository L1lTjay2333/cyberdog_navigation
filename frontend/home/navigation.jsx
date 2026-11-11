import React from 'react';
import { createRoot } from 'react-dom/client';
import Topography from './Topography';

class BackgroundBoundary extends React.Component {
  constructor(props) {
    super(props);
    this.state = { failed: false };
  }
  static getDerivedStateFromError() {
    return { failed: true };
  }
  render() {
    return this.state.failed ? null : this.props.children;
  }
}

createRoot(document.getElementById('navigation-background')).render(
  <BackgroundBoundary>
    <Topography
      color="#4f7cff"
      dpr={1}
      lineCount={22}
      lineWidth={1.2}
      elevation={1}
      foreground={0.2}
      tilt={32}
      speed={0} morph={0} glitch={0} markers={-1}
      direction="forward"
      glow={0.6}
      depthOfField={0.4}
      fog={0.35}
      hoverEffect="none"
      clickEffect="none"
    />
  </BackgroundBoundary>
);

