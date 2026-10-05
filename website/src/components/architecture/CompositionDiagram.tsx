import Link from '@docusaurus/Link';
import React, {useState} from 'react';
import styles from './architecture.module.css';

type Box = {
  id: string;
  label: string;
  sublabel: string;
  href: string;
  color: 'model' | 'ir' | 'compile' | 'runtime';
};

const STACK: Box[] = [
  {
    id: 'coord',
    label: 'restock_action_coordinator',
    sublabel: '/restock_product · admission gate',
    href: '/docs/architecture/task-execution',
    color: 'model',
  },
  {
    id: 'ws',
    label: 'world_state_node',
    sublabel: 'snapshots · reservations · commits',
    href: '/docs/architecture/world-state',
    color: 'ir',
  },
  {
    id: 'proj',
    label: 'planning_scene_projector + move_group',
    sublabel: 'scene diffs · OMPL · execution',
    href: '/docs/architecture/motion-planning',
    color: 'compile',
  },
  {
    id: 'ctrl',
    label: 'controllers + Gazebo + attachment',
    sublabel: 'ros2_control · gz_ros2_control · plugin',
    href: '/docs/architecture/simulation',
    color: 'runtime',
  },
];

const COLOR_CLASS: Record<Box['color'], string> = {
  model: styles.layerModel,
  ir: styles.layerIr,
  compile: styles.layerCompile,
  runtime: styles.layerRuntime,
};

export default function CompositionDiagram(): React.ReactElement {
  const [activeId, setActiveId] = useState<string | null>(null);
  return (
    <div className={styles.pipelineWrap}>
      <div className={styles.pipelineTitle}>just launch-baseline composition</div>
      <div className={styles.pipeline}>
        {STACK.map((box, i) => (
          <React.Fragment key={box.id}>
            {i > 0 && <div className={styles.arrow} />}
            <Link
              href={box.href}
              className={`${styles.layerBox} ${COLOR_CLASS[box.color]} ${
                activeId === box.id ? styles.layerBoxActive : ''
              }`}
              style={{width: 400}}
              onMouseEnter={() => setActiveId(box.id)}
              onMouseLeave={() => setActiveId(null)}>
              <span className={styles.layerLabel}>{box.label}</span>
              <span className={styles.layerSublabel}>{box.sublabel}</span>
            </Link>
          </React.Fragment>
        ))}
      </div>
      <div className={styles.legend}>
        Enabled by default: wrist perception · lane survey · depth obstacles.
        Opt-in: coordinated tray survey / campaign · overhead object perception · reasoner.
      </div>
    </div>
  );
}
