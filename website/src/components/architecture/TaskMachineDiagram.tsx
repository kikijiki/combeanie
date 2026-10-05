import React, {useState} from 'react';
import styles from './architecture.module.css';

type Phase = {
  id: string;
  label: string;
  sublabel: string;
  color: 'model' | 'ir' | 'compile' | 'runtime';
};

const PHASES: Phase[] = [
  {id: 'select', label: 'Validate · Select · Reserve', sublabel: 'world-state authority', color: 'model'},
  {id: 'grasp', label: 'Grasp plan → execute → close → verify', sublabel: 'candidates + MotionPort + GripperPort', color: 'ir'},
  {id: 'attach', label: 'Attach · retract · placement · carry start', sublabel: 'AttachmentPort scopes', color: 'compile'},
  {id: 'insert', label: 'Insert · open · physical detach · retreat · survey', sublabel: 'physical then semantic detach', color: 'runtime'},
  {id: 'done', label: 'Semantic commit · verify placement · inventory · complete', sublabel: 'or Recover / Fault / Operator', color: 'model'},
];

const COLOR_CLASS: Record<Phase['color'], string> = {
  model: styles.layerModel,
  ir: styles.layerIr,
  compile: styles.layerCompile,
  runtime: styles.layerRuntime,
};

export default function TaskMachineDiagram(): React.ReactElement {
  const [activeId, setActiveId] = useState<string | null>(null);
  return (
    <div className={styles.pipelineWrap}>
      <div className={styles.pipelineTitle}>Restock task machine (happy path)</div>
      <div className={styles.pipeline}>
        {PHASES.map((phase, i) => (
          <React.Fragment key={phase.id}>
            {i > 0 && <div className={styles.arrow} />}
            <div
              className={`${styles.layerBox} ${COLOR_CLASS[phase.color]} ${
                activeId === phase.id ? styles.layerBoxActive : ''
              }`}
              style={{width: 400, cursor: 'default'}}
              onMouseEnter={() => setActiveId(phase.id)}
              onMouseLeave={() => setActiveId(null)}>
              <span className={styles.layerLabel}>{phase.label}</span>
              <span className={styles.layerSublabel}>{phase.sublabel}</span>
            </div>
          </React.Fragment>
        ))}
      </div>
      <div className={styles.legend}>
        <span className={`${styles.legendDot} ${styles.layerModel}`} />Authority
        <span className={`${styles.legendDot} ${styles.layerIr}`} />Approach
        <span className={`${styles.legendDot} ${styles.layerCompile}`} />Carry
        <span className={`${styles.legendDot} ${styles.layerRuntime}`} />Place
      </div>
    </div>
  );
}
