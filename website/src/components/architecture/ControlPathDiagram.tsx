import Link from '@docusaurus/Link';
import React, {useState} from 'react';
import styles from './architecture.module.css';

type Layer = {
  id: string;
  label: string;
  sublabel?: string;
  href?: string;
  color: 'model' | 'ir' | 'compile' | 'runtime';
};

const STEPS: Layer[] = [
  {
    id: 'task',
    label: 'Task state machine',
    sublabel: 'RestockProduct action · coordinator driver',
    href: '/docs/architecture/task-execution',
    color: 'model',
  },
  {
    id: 'world',
    label: 'Authoritative world state',
    sublabel: 'identity · freshness · reservations · commits',
    href: '/docs/architecture/world-state',
    color: 'ir',
  },
  {
    id: 'candidates',
    label: 'Grasp & placement candidates',
    sublabel: 'deterministic geometry · scored batches',
    href: '/docs/architecture/manipulation',
    color: 'ir',
  },
  {
    id: 'moveit',
    label: 'MoveIt planning + collision',
    sublabel: 'planning scene projection · OMPL',
    href: '/docs/architecture/motion-planning',
    color: 'compile',
  },
  {
    id: 'exec',
    label: 'Trajectory execution',
    sublabel: 'motion port · gripper · attachment',
    href: '/docs/architecture/ports',
    color: 'compile',
  },
  {
    id: 'control',
    label: 'ros2_control',
    sublabel: 'rail · arm · gripper controllers',
    href: '/docs/architecture/simulation',
    color: 'runtime',
  },
  {
    id: 'gazebo',
    label: 'Gazebo joints & physics',
    sublabel: 'Harmonic · gz_ros2_control · attachment plugin',
    href: '/docs/architecture/simulation',
    color: 'runtime',
  },
];

const COLOR_CLASS: Record<Layer['color'], string> = {
  model: styles.layerModel,
  ir: styles.layerIr,
  compile: styles.layerCompile,
  runtime: styles.layerRuntime,
};

function LayerBox({
  layer,
  active,
  onHover,
}: {
  layer: Layer;
  active: boolean;
  onHover: (id: string | null) => void;
}) {
  return (
    <Link
      href={layer.href}
      className={`${styles.layerBox} ${COLOR_CLASS[layer.color]} ${active ? styles.layerBoxActive : ''}`}
      onMouseEnter={() => onHover(layer.id)}
      onMouseLeave={() => onHover(null)}>
      <span className={styles.layerLabel}>{layer.label}</span>
      {layer.sublabel && <span className={styles.layerSublabel}>{layer.sublabel}</span>}
    </Link>
  );
}

export default function ControlPathDiagram(): React.ReactElement {
  const [activeId, setActiveId] = useState<string | null>(null);

  return (
    <div className={styles.pipelineWrap}>
      <div className={styles.pipelineTitle}>Authoritative control path</div>
      <div className={styles.pipeline}>
        {STEPS.map((step, i) => (
          <React.Fragment key={step.id}>
            {i > 0 && <div className={styles.arrow} />}
            <LayerBox layer={step} active={activeId === step.id} onHover={setActiveId} />
          </React.Fragment>
        ))}
      </div>
      <div className={styles.legend}>
        <span className={`${styles.legendDot} ${styles.layerModel}`} />Task
        <span className={`${styles.legendDot} ${styles.layerIr}`} />Semantics
        <span className={`${styles.legendDot} ${styles.layerCompile}`} />Planning
        <span className={`${styles.legendDot} ${styles.layerRuntime}`} />Actuation
      </div>
    </div>
  );
}
