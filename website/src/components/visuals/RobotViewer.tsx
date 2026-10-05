import React from 'react';
import GlbViewer from './GlbViewer';

type Props = {
  caption?: string;
  height?: number;
};

export default function RobotViewer({
  caption = 'Rail-mounted UR10e with official meshes and the project gripper.',
  height,
}: Props) {
  return <GlbViewer kind="robot" caption={caption} height={height} />;
}
