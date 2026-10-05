import Link from '@docusaurus/Link';
import React, {useState} from 'react';
import styles from './architecture.module.css';

type Node = {
  id: string;
  label: string;
  sublabel?: string;
  href?: string;
  color: 'model' | 'ir' | 'compile' | 'runtime';
};

const ROBOT: Node[] = [
  {id: 'rail', label: 'rail_base → carriage → base_link', sublabel: 'prismatic rail + UR10e root', color: 'ir'},
  {id: 'arm', label: 'shoulder_link → … → wrist_3_link → tool0', sublabel: 'official ur_description chain', color: 'compile'},
  {id: 'grasp', label: 'tool0 children: gripper · wrist_camera_link', sublabel: 'gripper → grasp_center / fingers; camera → optical frame', color: 'runtime'},
];

const COLOR_CLASS: Record<Node['color'], string> = {
  model: styles.layerModel,
  ir: styles.layerIr,
  compile: styles.layerCompile,
  runtime: styles.layerRuntime,
};

function Box({
  node,
  active,
  onHover,
}: {
  node: Node;
  active: boolean;
  onHover: (id: string | null) => void;
}) {
  const className = `${styles.layerBox} ${COLOR_CLASS[node.color]} ${active ? styles.layerBoxActive : ''}`;
  const body = (
    <>
      <span className={styles.layerLabel}>{node.label}</span>
      {node.sublabel && <span className={styles.layerSublabel}>{node.sublabel}</span>}
    </>
  );
  if (node.href) {
    return (
      <Link
        href={node.href}
        className={className}
        style={{width: 280}}
        onMouseEnter={() => onHover(node.id)}
        onMouseLeave={() => onHover(null)}>
        {body}
      </Link>
    );
  }
  return (
    <div
      className={className}
      style={{width: 280, cursor: 'default'}}
      onMouseEnter={() => onHover(node.id)}
      onMouseLeave={() => onHover(null)}>
      {body}
    </div>
  );
}

function Column({
  title,
  nodes,
  activeId,
  onHover,
}: {
  title: string;
  nodes: Node[];
  activeId: string | null;
  onHover: (id: string | null) => void;
}) {
  return (
    <div className={styles.pipeline} style={{flex: 1}}>
      <div className={styles.pipelineTitle}>{title}</div>
      {nodes.map((n, i) => (
        <React.Fragment key={n.id}>
          {i > 0 && <div className={styles.arrow} />}
          <Box node={n} active={activeId === n.id} onHover={onHover} />
        </React.Fragment>
      ))}
    </div>
  );
}

export default function FrameTreeDiagram(): React.ReactElement {
  const [activeId, setActiveId] = useState<string | null>(null);
  return (
    <div className={styles.pipelineWrap}>
      <div className={styles.pipelineTitle}>Coordinate frame tree · world (metres, REP-103)</div>
      <p style={{textAlign: 'center'}}>Both branches start at world. Arrows show ancestry; grouped siblings do not form a chain.</p>
      <div className={styles.forkRow} style={{alignItems: 'flex-start', justifyContent: 'center', gap: '2rem'}}>
        <Column title="Robot branch" nodes={ROBOT} activeId={activeId} onHover={setActiveId} />
        <div className={styles.pipeline} style={{flex: 1}}>
          <div className={styles.pipelineTitle}>Shelf branch</div>
          <Box node={{id: 'shelf', label: 'shelf', sublabel: 'static world → shelf transform', color: 'model'}} active={false} onHover={setActiveId} />
          <div className={styles.arrow} />
          <div className={styles.pipelineTitle}>Separate children of shelf</div>
          <Box node={{id: 'lanes', label: 'lane_01 … lane_06', sublabel: 'six sibling frames · +Y downhill', color: 'ir'}} active={false} onHover={setActiveId} />
          <div style={{height: '1rem'}} />
          <Box node={{id: 'bed', label: 'roller_bed', sublabel: 'fixed shelf child', color: 'ir'}} active={false} onHover={setActiveId} />
          <div style={{height: '1rem'}} />
          <Box node={{id: 'camera', label: 'overhead_camera_link', sublabel: 'fixed shelf child', color: 'compile'}} active={false} onHover={setActiveId} />
          <div className={styles.arrow} />
          <Box node={{id: 'optical', label: 'overhead_camera_optical_frame', sublabel: 'child of camera link · image stamps', color: 'runtime'}} active={false} onHover={setActiveId} />
        </div>
      </div>
      <div className={styles.legend}>
        <span className={`${styles.legendDot} ${styles.layerModel}`} />Root
        <span className={`${styles.legendDot} ${styles.layerIr}`} />Structure
        <span className={`${styles.legendDot} ${styles.layerCompile}`} />Kinematics / sensors
        <span className={`${styles.legendDot} ${styles.layerRuntime}`} />End-effector
      </div>
    </div>
  );
}
