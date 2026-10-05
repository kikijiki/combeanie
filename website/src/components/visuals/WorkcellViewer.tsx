import React from 'react';
import GlbViewer from './GlbViewer';

type Props = {
  caption?: string;
  height?: number;
};

export default function WorkcellViewer({
  caption = 'Gravity-fed shelf, six lanes, stock tray, and overhead camera.',
  height,
}: Props) {
  return <GlbViewer kind="workcell" caption={caption} height={height} />;
}
