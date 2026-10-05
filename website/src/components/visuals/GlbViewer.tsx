import React, {lazy, Suspense, useState} from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './visuals.module.css';

export type GlbViewerKind = 'robot' | 'workcell' | 'combined';
type Props = {kind: GlbViewerKind; caption?: string; height?: number; eager?: boolean};
const GlbViewerCanvas = lazy(() => import('./GlbViewerCanvas'));
const SCHEMATIC = {
  robot: 'robot-orthographic.svg',
  workcell: 'workcell-top-side.svg',
  combined: 'layout-plan.svg',
};

export function StaticViewer({kind, message, children}: {
  kind: GlbViewerKind; message: string; children?: React.ReactNode;
}) {
  const src = useBaseUrl(`/img/schematics/${SCHEMATIC[kind]}`);
  return <div className={styles.fallback}>
    <img src={src} alt={`${kind} schematic; geometry and frame details are described in the surrounding text`} />
    <p role="status">{message}</p>
    {children}
  </div>;
}

class ViewerBoundary extends React.Component<React.PropsWithChildren<{kind: GlbViewerKind}>, {failed: boolean}> {
  state = {failed: false};
  static getDerivedStateFromError() { return {failed: true}; }
  render() {
    return this.state.failed
      ? <StaticViewer kind={this.props.kind} message="Interactive viewer unavailable. Use this static schematic." />
      : this.props.children;
  }
}

function InteractiveViewer({kind, height, eager}: {kind: GlbViewerKind; height: number; eager: boolean}) {
  const [active, setActive] = useState(eager);
  if (!active) return <StaticViewer kind={kind} message="Static schematic. Load the 3D model to orbit and zoom.">
    <button type="button" className={styles.toolBtn} onClick={() => setActive(true)}>Load interactive {kind} model</button>
  </StaticViewer>;
  return <ViewerBoundary kind={kind}>
    <Suspense fallback={<StaticViewer kind={kind} message="Loading interactive model…" />}>
      <GlbViewerCanvas kind={kind} height={height} />
    </Suspense>
  </ViewerBoundary>;
}

export default function GlbViewer({kind, caption, height = 420, eager = false}: Props) {
  return <figure className={styles.figure}>
    <BrowserOnly fallback={<StaticViewer kind={kind} message="Static schematic. JavaScript and WebGL enable the optional interactive view." />}>
      {() => <InteractiveViewer kind={kind} height={height} eager={eager} />}
    </BrowserOnly>
    {caption ? <figcaption className={styles.caption}>{caption}</figcaption> : null}
  </figure>;
}
