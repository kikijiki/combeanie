import Link from '@docusaurus/Link';
import React, {useMemo, useState} from 'react';
import styles from './architecture.module.css';

type Entry = {
  term: string;
  def: string;
  see?: string;
};

const ENTRIES: Entry[] = [
  {
    term: 'AttachmentPort',
    def: 'Coordinator port that attaches or detaches a product across Gazebo, the planning scene, and world-state semantics. Supports Full / PhysicalOnly / SemanticOnly scopes for place-detach splits.',
    see: '/docs/architecture/ports',
  },
  {
    term: 'ColourDepthBackend',
    def: 'Classical RGB-D perception backend: chromaticity segmentation plus cylinder depth fit. Not a neural net. Default camera producer when perception:=true.',
    see: '/docs/architecture/perception-and-models',
  },
  {
    term: 'DoF',
    def: 'Degree of freedom: an independent joint that can be commanded. The restocker has a prismatic rail plus a six-joint arm and a parallel-jaw gripper.',
    see: '/docs/architecture/workcell-and-frames',
  },
  {
    term: 'Fail closed',
    def: 'Policy that stops or refuses rather than guessing when evidence is stale, reservations are orphaned, or outcomes cannot be proven.',
    see: '/docs/architecture/task-execution',
  },
  {
    term: 'FollowJointTrajectory',
    def: 'Action interface ros2_control trajectory controllers expose. MoveIt execution sends timed joint paths through this interface into Gazebo via gz_ros2_control.',
    see: '/docs/architecture/simulation',
  },
  {
    term: 'grasp_center',
    def: 'Named frame on the gripper used as the semantic grasp target. Candidates are computed there, then converted to tool0 goals for the kinematic chain.',
    see: '/docs/architecture/workcell-and-frames',
  },
  {
    term: 'GripperPort',
    def: 'Coordinator port that commands one jaw opening and independently verifies finger positions against the attachment band (0.5 mm), not the controller’s looser goal tolerance.',
    see: '/docs/architecture/ports',
  },
  {
    term: 'gz_ros2_control',
    def: 'Gazebo system plugin that presents simulated joints as a ros2_control hardware interface so the same controllers work in simulation.',
    see: '/docs/architecture/simulation',
  },
  {
    term: 'Lane contents',
    def: 'Semantic membership of products in a shelf lane. Added by reservation-authorized placement commits; accepted lane surveys can reconcile FIFO sales and remove entries. Raw observations cannot directly commit inventory.',
    see: '/docs/architecture/world-state',
  },
  {
    term: 'MotionPort',
    def: 'Coordinator port that plans and executes one trajectory segment in a single submission. Outcomes classify “nothing commanded” versus “arm may have moved.”',
    see: '/docs/architecture/ports',
  },
  {
    term: 'OMPL',
    def: 'Open Motion Planning Library. Baseline planners (RRTConnect) used by MoveIt. Accelerated planners stay out of the default pipeline until they plug in through the same MoveIt interface.',
    see: '/docs/architecture/motion-planning',
  },
  {
    term: 'Optical frame',
    def: 'Camera frame with image-aligned axes per REP-103 (+Z forward, +X right, +Y down). Image and CameraInfo topics stamp optical frames, not body links.',
    see: '/docs/architecture/workcell-and-frames',
  },
  {
    term: 'Planning scene',
    def: 'MoveIt’s collision world: robot state plus collision objects. Geometry for safety, not a product database. Fed by the planning-scene projector.',
    see: '/docs/architecture/motion-planning',
  },
  {
    term: 'Planning-scene projector',
    def: 'Node that copies enough geometry from world state and depth obstacle evidence (enabled by default) into MoveIt under leases, then gates motion on APPLIED status + revision.',
    see: '/docs/architecture/motion-planning',
  },
  {
    term: 'Reasoner',
    def: 'Optional advisory recovery client (HTTP, strict JSON). May only select among enumerated recovery primitives and never escalate aggressiveness. Off by default.',
    see: '/docs/architecture/perception-and-models',
  },
  {
    term: 'Reservation token',
    def: 'Opaque capability returned by world-state reserve services. Required for semantic attachment/detachment commits. Never logged or snapshotted; orphaned tokens inhibit admission.',
    see: '/docs/architecture/world-state',
  },
  {
    term: 'RestockProduct',
    def: 'Public action API. Optional object/lane selectors; feedback carries task state; result is a typed terminal status judged against world state.',
    see: '/docs/architecture/task-execution',
  },
  {
    term: 'RRTConnect',
    def: 'Bidirectional rapidly exploring random tree planner. Default OMPL planner for the manipulator group in this project.',
    see: '/docs/architecture/motion-planning',
  },
  {
    term: 'SRDF',
    def: 'Semantic Robot Description Format for MoveIt: planning groups, disabled collisions, and end-effector definitions.',
    see: '/docs/reference/packages',
  },
  {
    term: 'tool0',
    def: 'Nominal tool-tip parent frame on the flange chain. MoveIt solves end-effector goals to tool0; grasp planning still targets grasp_center semantically.',
    see: '/docs/architecture/workcell-and-frames',
  },
  {
    term: 'World state',
    def: 'Authoritative C++ store for product identity, lane membership, robot execution semantics, revisions, and reservations. Perception refreshes evidence; authorized commits add contents and accepted surveys reconcile FIFO sales.',
    see: '/docs/architecture/world-state',
  },
  {
    term: 'WorldStatePort',
    def: 'Coordinator port for snapshots, reservations, validation, and semantic attach/detach commits. Mutation replies carry opaque tokens.',
    see: '/docs/architecture/ports',
  },
].sort((a, b) => a.term.localeCompare(b.term));

const ALPHABET = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'.split('');

export default function GlossaryComponent(): React.ReactElement {
  const [query, setQuery] = useState('');
  const q = query.toLowerCase();

  const filtered = q
    ? ENTRIES.filter(
        (e) => e.term.toLowerCase().includes(q) || e.def.toLowerCase().includes(q),
      )
    : ENTRIES;

  const byLetter = useMemo(() => {
    const map: Record<string, Entry[]> = {};
    for (const e of filtered) {
      const l = e.term[0].toUpperCase();
      if (!map[l]) map[l] = [];
      map[l].push(e);
    }
    return map;
  }, [filtered]);

  const usedLetters = new Set(Object.keys(byLetter));

  return (
    <div className={styles.glossaryWrap}>
      <label htmlFor="glossary-search">Search glossary</label>
      <input
        id="glossary-search"
        type="search"
        className={styles.glossarySearch}
        placeholder="Search terms..."
        value={query}
        onChange={(e) => setQuery(e.target.value)}
      />
      {!q && (
        <div className={styles.glossaryJumps}>
          {ALPHABET.map((l) => (
            <button
              key={l}
              type="button"
              disabled={!usedLetters.has(l)}
              className={`${styles.glossaryJump} ${usedLetters.has(l) ? '' : styles.glossaryJumpDisabled}`}
              onClick={() =>
                usedLetters.has(l) &&
                document.getElementById(`gloss-${l}`)?.scrollIntoView({behavior: 'smooth'})
              }>
              {l}
            </button>
          ))}
        </div>
      )}
      {Object.keys(byLetter)
        .sort()
        .map((letter) => (
          <div key={letter} className={styles.glossaryGroup} id={`gloss-${letter}`}>
            <div className={styles.glossaryGroupLetter}>{letter}</div>
            {byLetter[letter].map((entry) => (
              <div key={entry.term} className={styles.glossaryEntry}>
                <span className={styles.glossaryTerm}>{entry.term}</span>
                <span className={styles.glossaryDef}>{entry.def}</span>
                {entry.see && (
                  <Link className={styles.glossaryLink} to={entry.see}>
                    see {entry.see.split('/').pop()?.replace(/-/g, ' ')}
                  </Link>
                )}
              </div>
            ))}
          </div>
        ))}
      {filtered.length === 0 && (
        <p style={{color: 'var(--ifm-color-emphasis-500)'}}>No terms match.</p>
      )}
    </div>
  );
}
