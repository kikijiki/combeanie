import Link from '@docusaurus/Link';
import React from 'react';
import styles from './architecture.module.css';

type Box = {
  id: string;
  label: string;
  sub?: string;
  href: string;
  x: number;
  y: number;
  w: number;
  h: number;
  fill: string;
};

const BOXES: Box[] = [
  {
    id: 'client',
    label: 'RestockProduct client',
    href: '/docs/architecture/task-execution',
    x: 200,
    y: 12,
    w: 240,
    h: 42,
    fill: 'var(--stack-fill-task)',
  },
  {
    id: 'task',
    label: 'Task state machine',
    href: '/docs/architecture/task-execution',
    x: 200,
    y: 78,
    w: 240,
    h: 46,
    fill: 'var(--stack-fill-task)',
  },
  {
    id: 'advise',
    label: 'Advisory models',
    sub: 'optional',
    href: '/docs/fundamentals/classical-vs-learned',
    x: 470,
    y: 78,
    w: 150,
    h: 46,
    fill: 'var(--stack-fill-advise)',
  },
  {
    id: 'world',
    label: 'World state',
    sub: 'snapshots · reserve · commit',
    href: '/docs/architecture/world-state',
    x: 36,
    y: 168,
    w: 190,
    h: 54,
    fill: 'var(--stack-fill-sem)',
  },
  {
    id: 'geom',
    label: 'Grasp / place geometry',
    href: '/docs/architecture/manipulation',
    x: 260,
    y: 168,
    w: 190,
    h: 54,
    fill: 'var(--stack-fill-sem)',
  },
  {
    id: 'moveit',
    label: 'MoveIt',
    sub: 'OMPL + execution',
    href: '/docs/architecture/motion-planning',
    x: 260,
    y: 256,
    w: 190,
    h: 50,
    fill: 'var(--stack-fill-plan)',
  },
  {
    id: 'ctrl',
    label: 'ros2_control',
    href: '/docs/fundamentals/gazebo-and-control',
    x: 260,
    y: 338,
    w: 190,
    h: 42,
    fill: 'var(--stack-fill-act)',
  },
  {
    id: 'obs',
    label: 'Observations',
    sub: 'perception or ground truth',
    href: '/docs/fundamentals/perception',
    x: 36,
    y: 338,
    w: 190,
    h: 54,
    fill: 'var(--stack-fill-advise)',
  },
  {
    id: 'gzhw',
    label: 'gz_ros2_control',
    href: '/docs/fundamentals/gazebo-and-control',
    x: 260,
    y: 412,
    w: 190,
    h: 42,
    fill: 'var(--stack-fill-act)',
  },
  {
    id: 'gazebo',
    label: 'Gazebo physics',
    href: '/docs/architecture/simulation',
    x: 260,
    y: 486,
    w: 190,
    h: 46,
    fill: 'var(--stack-fill-act)',
  },
  {
    id: 'tf',
    label: 'TF · joint_states · cameras · /clock',
    href: '/docs/fundamentals/urdf-and-tf',
    x: 36,
    y: 560,
    w: 414,
    h: 42,
    fill: 'var(--stack-fill-tf)',
  },
];

type Edge = {
  from: string;
  to: string;
  label?: string;
  labelX?: number;
  labelY?: number;
  dashed?: boolean;
  d: string;
};

// Explicit paths so layout stays readable.
const EDGES: Edge[] = [
  {from: 'client', to: 'task', d: 'M 320 54 L 320 78'},
  {
    from: 'task',
    to: 'advise',
    label: 'recovery choice only',
    labelX: 455,
    labelY: 70,
    dashed: true,
    d: 'M 440 101 L 470 101',
  },
  {
    from: 'task',
    to: 'world',
    label: 'reserve / commit',
    labelX: 150,
    labelY: 140,
    d: 'M 260 124 L 260 140 L 131 140 L 131 168',
  },
  {
    from: 'task',
    to: 'geom',
    label: 'candidates',
    labelX: 390,
    labelY: 148,
    d: 'M 355 124 L 355 168',
  },
  {from: 'geom', to: 'moveit', d: 'M 355 222 L 355 256'},
  {from: 'moveit', to: 'ctrl', d: 'M 355 306 L 355 338'},
  {from: 'ctrl', to: 'gzhw', d: 'M 355 380 L 355 412'},
  {from: 'gzhw', to: 'gazebo', d: 'M 355 454 L 355 486'},
  {from: 'obs', to: 'world', d: 'M 131 338 L 131 222'},
  {from: 'obs', to: 'tf', d: 'M 100 392 L 100 560'},
  {from: 'gazebo', to: 'tf', d: 'M 355 532 L 355 548 L 243 548 L 243 560'},
];

export default function StackDependencyDiagram(): React.ReactElement {
  return (
    <div className={styles.pipelineWrap}>
      <div className={styles.pipelineTitle}>Who depends on whom</div>
      <div className={styles.stackSvgWrap}>
        <svg
          className={styles.stackSvg}
          viewBox="0 0 640 620"
          role="img"
          aria-label="Dependency diagram from RestockProduct client down to Gazebo and TF">
          <defs>
            <marker
              id="stackArrow"
              viewBox="0 0 10 10"
              refX="9"
              refY="5"
              markerWidth="7"
              markerHeight="7"
              orient="auto-start-reverse">
              <path d="M 0 0 L 10 5 L 0 10 z" className={styles.stackArrowHead} />
            </marker>
          </defs>

          {EDGES.map((e) => (
            <g key={`${e.from}-${e.to}`}>
              <path
                d={e.d}
                className={e.dashed ? styles.stackEdgeDashed : styles.stackEdge}
                markerEnd="url(#stackArrow)"
              />
              {e.label && e.labelX != null && e.labelY != null && (
                <text x={e.labelX} y={e.labelY} className={styles.stackEdgeLabel} textAnchor="middle">
                  {e.label}
                </text>
              )}
            </g>
          ))}

          {BOXES.map((b) => (
            <Link key={b.id} href={b.href} className={styles.stackBoxLink}>
              <rect
                x={b.x}
                y={b.y}
                width={b.w}
                height={b.h}
                rx={6}
                className={styles.stackBox}
                style={{fill: b.fill}}
              />
              <text
                x={b.x + b.w / 2}
                y={b.sub ? b.y + b.h / 2 - 6 : b.y + b.h / 2 + 4}
                textAnchor="middle"
                className={styles.stackBoxLabel}>
                {b.label}
              </text>
              {b.sub && (
                <text
                  x={b.x + b.w / 2}
                  y={b.y + b.h / 2 + 12}
                  textAnchor="middle"
                  className={styles.stackBoxSub}>
                  {b.sub}
                </text>
              )}
            </Link>
          ))}
        </svg>
      </div>
      <div className={styles.legend}>
        <span className={`${styles.legendDot} ${styles.layerModel}`} />Task
        <span className={`${styles.legendDot} ${styles.layerIr}`} />Semantics / advise
        <span className={`${styles.legendDot} ${styles.layerCompile}`} />Planning
        <span className={`${styles.legendDot} ${styles.layerRuntime}`} />Actuation
        <span className={styles.legendNote}>Dashed: optional advisory path</span>
      </div>
    </div>
  );
}
