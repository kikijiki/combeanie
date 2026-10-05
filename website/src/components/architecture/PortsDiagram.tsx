import Link from '@docusaurus/Link';
import React, {useState} from 'react';
import styles from './architecture.module.css';

type Port = {
  id: string;
  label: string;
  sublabel: string;
  href: string;
  color: 'model' | 'ir' | 'compile' | 'runtime';
};

const DRIVER: Port = {
  id: 'driver',
  label: 'RestockCoordinatorDriver',
  sublabel: 'pump · inbox · admission · fault epoch',
  href: '/docs/architecture/task-execution',
  color: 'model',
};

const PORTS: Port[] = [
  {
    id: 'world',
    label: 'WorldStatePort',
    sublabel: 'snapshots · reserve · commit · release',
    href: '/docs/architecture/world-state',
    color: 'ir',
  },
  {
    id: 'motion',
    label: 'MotionPort',
    sublabel: 'plan + execute one segment',
    href: '/docs/architecture/ports',
    color: 'compile',
  },
  {
    id: 'gripper',
    label: 'GripperPort',
    sublabel: 'jaw opening · independent verify',
    href: '/docs/architecture/ports',
    color: 'compile',
  },
  {
    id: 'attach',
    label: 'AttachmentPort',
    sublabel: 'Gazebo + scene + semantics',
    href: '/docs/architecture/ports',
    color: 'runtime',
  },
];

const COLOR_CLASS: Record<Port['color'], string> = {
  model: styles.layerModel,
  ir: styles.layerIr,
  compile: styles.layerCompile,
  runtime: styles.layerRuntime,
};

function Box({
  port,
  active,
  onHover,
  wide,
}: {
  port: Port;
  active: boolean;
  onHover: (id: string | null) => void;
  wide?: boolean;
}) {
  return (
    <Link
      href={port.href}
      className={`${styles.layerBox} ${COLOR_CLASS[port.color]} ${active ? styles.layerBoxActive : ''}`}
      style={wide ? {width: 420} : {width: 200}}
      onMouseEnter={() => onHover(port.id)}
      onMouseLeave={() => onHover(null)}>
      <span className={styles.layerLabel}>{port.label}</span>
      <span className={styles.layerSublabel}>{port.sublabel}</span>
    </Link>
  );
}

export default function PortsDiagram(): React.ReactElement {
  const [activeId, setActiveId] = useState<string | null>(null);

  return (
    <div className={styles.pipelineWrap}>
      <div className={styles.pipelineTitle}>Coordinator ports</div>
      <div className={styles.pipeline}>
        <Box port={DRIVER} active={activeId === DRIVER.id} onHover={setActiveId} wide />
        <div className={styles.arrow} />
        <div className={styles.forkRow} style={{flexWrap: 'wrap', justifyContent: 'center'}}>
          {PORTS.map((p) => (
            <div key={p.id} className={styles.forkBranch}>
              <Box port={p} active={activeId === p.id} onHover={setActiveId} />
            </div>
          ))}
        </div>
      </div>
      <div className={styles.legend}>
        <span className={`${styles.legendDot} ${styles.layerModel}`} />Driver
        <span className={`${styles.legendDot} ${styles.layerIr}`} />Semantics
        <span className={`${styles.legendDot} ${styles.layerCompile}`} />Motion
        <span className={`${styles.legendDot} ${styles.layerRuntime}`} />Coupling
      </div>
    </div>
  );
}
