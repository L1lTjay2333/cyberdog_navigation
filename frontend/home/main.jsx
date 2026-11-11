import './HomeFolders';
import React from 'react';
import { createRoot } from 'react-dom/client';
import DitherVeil from './DitherVeil';

class BackgroundBoundary extends React.Component {
  constructor(props) {
    super(props);
    this.state = { failed: false };
  }
  static getDerivedStateFromError() {
    return { failed: true };
  }
  render() {
    if (this.state.failed) {
      return <div style={{ width: '100%', height: '100%', background: '#f4f1ea' }} />;
    }
    return this.props.children;
  }
}

createRoot(document.getElementById('dither-background')).render(
  <BackgroundBoundary>
    <DitherVeil
      src="/assets/home/dog-color.png"
      matteColor="#ffffff"
      fit="cover"
      imageScale={0.70}
      imageOffsetX={0.28}
      clickBurst={false}
      pattern="floyd"
      pixelSize={2}
      inkColor="#120f17"
      paperColor="#f4f1ea"
      revealRadius={200}
      softness={0.6}
      linger={1}
    />
  </BackgroundBoundary>
);
