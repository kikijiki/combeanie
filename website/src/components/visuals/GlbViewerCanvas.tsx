import React, {Suspense, useCallback, useEffect, useMemo, useRef, useState} from 'react';
import {Canvas, useThree} from '@react-three/fiber';
import {Html, OrbitControls, useGLTF} from '@react-three/drei';
import {StaticViewer, type GlbViewerKind} from './GlbViewer';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './visuals.module.css';
import * as THREE from 'three';

type OrbitControlsHandle = {
  target: THREE.Vector3;
  update: () => void;
};

const MODEL_URL: Record<GlbViewerKind, string> = {
  robot: '/models/robot.glb',
  workcell: '/models/workcell.glb',
  combined: '/models/restocking_cell.glb',
};

type PoseId = 'home' | 'reach';

type LabelDef = {id: string; text: string; node: string};

// These are visual-component labels, not TF origins. GLBs contain baked visual nodes.
// Resolve their centers from the exported transforms for the selected pose.
const LABELS: Record<GlbViewerKind, LabelDef[]> = {
  robot: [
    {id: 'rail', text: 'rail', node: 'rail_base__visual_0'},
    {id: 'carriage', text: 'carriage', node: 'carriage__visual_0'},
    {id: 'gripper', text: 'gripper body', node: 'gripper__visual_0'},
  ],
  workcell: [
    {id: 'shelf', text: 'roller bed', node: 'roller_bed'},
    {id: 'tray', text: 'stock tray', node: 'stock_tray'},
    {id: 'lane03', text: 'lane 03 marker', node: 'lane_marker_lane_03'},
    {id: 'cam', text: 'overhead camera body', node: 'overhead_camera'},
  ],
  combined: [
    {id: 'rail', text: 'rail', node: 'robot__rail_base__visual_0'},
    {id: 'shelf', text: 'roller bed', node: 'workcell__roller_bed'},
    {id: 'tray', text: 'stock tray', node: 'workcell__stock_tray'},
    {id: 'gripper', text: 'gripper body (reach)', node: 'robot__gripper__visual_0'},
  ],
};

const CAMERA: Record<GlbViewerKind, {position: [number, number, number]; target: [number, number, number]}> = {
  robot: {position: [3.2, 2.4, 3.2], target: [0, 1.0, 0]},
  workcell: {position: [3.4, 2.2, 2.8], target: [0, 0.4, 0]},
  combined: {position: [4.2, 2.8, 3.6], target: [0, 0.9, 0.2]},
};

function useThemeBg(): string {
  const [bg, setBg] = useState('#f4f6f8');
  useEffect(() => {
    const read = () => {
      const dark = document.documentElement.getAttribute('data-theme') === 'dark';
      setBg(dark ? '#1b1f24' : '#f4f6f8');
    };
    read();
    const obs = new MutationObserver(read);
    obs.observe(document.documentElement, {attributes: true, attributeFilter: ['data-theme']});
    return () => obs.disconnect();
  }, []);
  return bg;
}

function Model({
  url,
  pose,
  kind,
  activeId,
  onSelect,
}: {
  activeId: string | null;
  onSelect: (id: string) => void;
  url: string;
  pose: PoseId;
  kind: GlbViewerKind;
}) {
  const {scene} = useGLTF(url);
  const invalidate = useThree((state) => state.invalidate);
  const root = useMemo(() => scene.clone(true), [scene]);

  useEffect(() => {
    root.traverse((obj) => {
      const mesh = obj as THREE.Mesh;
      if (mesh.isMesh) {
        const mats = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
        for (const mat of mats) {
          if (mat && 'metalness' in mat) {
            const std = mat as THREE.MeshStandardMaterial;
            std.metalness = 0.15;
            std.roughness = 0.55;
            std.needsUpdate = true;
          }
        }
      }

      if (kind !== 'robot') {
        return;
      }
      const name = obj.name || '';
      if (name.startsWith('home__')) {
        obj.visible = pose === 'home';
      } else if (name.startsWith('reach__')) {
        obj.visible = pose === 'reach';
      }
    });
    invalidate();
  }, [root, pose, kind, invalidate]);

  // GLBs are authored in ROS/URDF frames (Z up). Three.js is Y up.
  return <group rotation={[-Math.PI / 2, 0, 0]}>
    <primitive object={root} />
    <Labels root={root} pose={pose} kind={kind} activeId={activeId} onSelect={onSelect} />
  </group>;
}

function Labels({
  root,
  pose,
  kind,
  activeId,
  onSelect,
}: {
  root: THREE.Object3D;
  pose: PoseId;
  kind: GlbViewerKind;
  activeId: string | null;
  onSelect: (id: string) => void;
}) {
  const labels = useMemo(() => {
    // Box3 includes the node's authored matrix and any baked mesh offsets. Keep positions
    // ROS-local here: the shared parent performs (x,y,z) -> (x,z,-y) exactly once.
    root.updateMatrixWorld(true);
    const inverseRoot = root.matrixWorld.clone().invert();
    return LABELS[kind].map((label) => {
      const node = root.getObjectByName(kind === 'robot' ? `${pose}__${label.node}` : label.node);
      if (!node) throw new Error(`Model is missing annotation node ${label.node}`);
      const position = new THREE.Box3().setFromObject(node).getCenter(new THREE.Vector3()).applyMatrix4(inverseRoot);
      return {...label, position};
    });
  }, [root, pose, kind]);
  return (
    <>
      {labels.map((label) => (
        <Html key={label.id} position={label.position} center distanceFactor={8} zIndexRange={[10, 0]}>
          <button
            type="button"
            className={`${styles.labelChip} ${activeId === label.id ? styles.labelChipActive : ''}`}
            aria-pressed={activeId === label.id}
            onClick={(e) => {
              e.stopPropagation();
              onSelect(label.id);
            }}>
            {label.text}
          </button>
        </Html>
      ))}
    </>
  );
}

function CameraRig({
  kind,
  controlsRef,
  resetToken,
}: {
  kind: GlbViewerKind;
  controlsRef: React.MutableRefObject<OrbitControlsHandle | null>;
  resetToken: number;
}) {
  const {camera} = useThree();
  const cfg = CAMERA[kind];

  useEffect(() => {
    camera.up.set(0, 1, 0);
    camera.position.set(...cfg.position);
    camera.lookAt(...cfg.target);
    camera.updateProjectionMatrix();
    const controls = controlsRef.current;
    if (controls) {
      controls.target.set(...cfg.target);
      controls.update();
    }
  }, [camera, cfg, controlsRef, resetToken, kind]);

  return (
    <OrbitControls
      ref={controlsRef as React.Ref<never>}
      makeDefault
      enableDamping
      dampingFactor={0.08}
      minDistance={1.2}
      maxDistance={14}
    />
  );
}

export type GlbViewerCanvasProps = {
  kind: GlbViewerKind;
  height: number;
};

export default function GlbViewerCanvas({kind, height}: GlbViewerCanvasProps) {
  const bg = useThemeBg();
  const controlsRef = useRef<OrbitControlsHandle | null>(null);
  const [resetToken, setResetToken] = useState(0);
  const [pose, setPose] = useState<PoseId>('home');
  const [activeLabel, setActiveLabel] = useState<string | null>(null);
  const url = useBaseUrl(MODEL_URL[kind]);

  const resetCamera = useCallback(() => {
    setResetToken((n) => n + 1);
  }, []);

  return (
    <div className={styles.viewerShell} style={{height}}>
      <div className={styles.toolbar}>
        <button type="button" className={styles.toolBtn} onClick={resetCamera}>
          Reset camera
        </button>
        {kind === 'robot' ? (
          <>
            <button
              type="button"
              className={`${styles.toolBtn} ${pose === 'home' ? styles.toolBtnActive : ''}`}
              aria-pressed={pose === 'home'}
              onClick={() => setPose('home')}>
              Home
            </button>
            <button
              type="button"
              className={`${styles.toolBtn} ${pose === 'reach' ? styles.toolBtnActive : ''}`}
              aria-pressed={pose === 'reach'}
              onClick={() => setPose('reach')}>
              Reach to lane
            </button>
          </>
        ) : null}
        {activeLabel ? (
          <span className={styles.activeLabel}>Selected: {LABELS[kind].find((l) => l.id === activeLabel)?.text}</span>
        ) : null}
      </div>
      <Canvas
        className={styles.canvas}
        frameloop="demand"
        fallback={<StaticViewer kind={kind} message="WebGL unavailable. Use this static schematic." />}
        dpr={[1, 1.75]}
        camera={{fov: 42, near: 0.05, far: 80, position: CAMERA[kind].position, up: [0, 1, 0]}}
        gl={{antialias: true, alpha: false}}
        onPointerMissed={() => setActiveLabel(null)}>
        <color attach="background" args={[bg]} />
        <ambientLight intensity={0.7} />
        <directionalLight position={[4, 8, 3]} intensity={1.05} />
        <directionalLight position={[-3, 4, -2]} intensity={0.35} />
        <hemisphereLight args={['#e8eef5', '#3a3f45', 0.45]} />
        <Suspense fallback={<Html center><span role="status">Loading model…</span></Html>}>
          <Model url={url} pose={pose} kind={kind} activeId={activeLabel} onSelect={setActiveLabel} />
        </Suspense>
        <CameraRig kind={kind} controlsRef={controlsRef} resetToken={resetToken} />
        {/* Ground plane for depth (theme-neutral); Three.js Y-up */}
        <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.02, 0]} receiveShadow>
          <planeGeometry args={[20, 20]} />
          <meshStandardMaterial color={bg === '#1b1f24' ? '#242a31' : '#e8ecf0'} roughness={1} metalness={0} />
        </mesh>
      </Canvas>
    </div>
  );
}
