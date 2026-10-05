import Link from '@docusaurus/Link';
import React, {useState} from 'react';
import styles from './architecture.module.css';

type Layer = {
  id: string;
  label: string;
  sublabel: string;
  href: string;
  color: 'model' | 'ir' | 'compile' | 'runtime';
};

const LAYERS: Layer[] = [
  {
    id: 'auth',
    label: 'Authoritative',
    sublabel: 'C++ world state · task machine · MoveIt / OMPL · recovery primitives',
    href: '/docs/architecture/world-state',
    color: 'model',
  },
  {
    id: 'proj',
    label: 'Projection (not inventory)',
    sublabel: 'planning_scene_projector → MoveIt collision geometry',
    href: '/docs/architecture/motion-planning',
    color: 'compile',
  },
  {
    id: 'adv',
    label: 'Advisory / untrusted inputs',
    sublabel: 'perception backends · reasoner · early GT as observation source',
    href: '/docs/architecture/perception-and-models',
    color: 'ir',
  },
  {
    id: 'phys',
    label: 'Physical coupling (simulator)',
    sublabel: 'Gazebo attachment plugin · multi-tick evidence',
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

export default function TrustBoundaryDiagram(): React.ReactElement {
  const [activeId, setActiveId] = useState<string | null>(null);
  return (
    <div className={styles.pipelineWrap}>
      <div className={styles.pipelineTitle}>Trust boundaries</div>
      <div className={styles.pipeline}>
        {LAYERS.map((layer, i) => (
          <React.Fragment key={layer.id}>
            {i > 0 && <div className={styles.arrow} />}
            <Link
              href={layer.href}
              className={`${styles.layerBox} ${COLOR_CLASS[layer.color]} ${
                activeId === layer.id ? styles.layerBoxActive : ''
              }`}
              style={{width: 420}}
              onMouseEnter={() => setActiveId(layer.id)}
              onMouseLeave={() => setActiveId(null)}>
              <span className={styles.layerLabel}>{layer.label}</span>
              <span className={styles.layerSublabel}>{layer.sublabel}</span>
            </Link>
          </React.Fragment>
        ))}
      </div>
      <div className={styles.legend}>
        <span className={`${styles.legendDot} ${styles.layerModel}`} />Decide
        <span className={`${styles.legendDot} ${styles.layerCompile}`} />Geometry view
        <span className={`${styles.legendDot} ${styles.layerIr}`} />Advise
        <span className={`${styles.legendDot} ${styles.layerRuntime}`} />Couple
      </div>
    </div>
  );
}
