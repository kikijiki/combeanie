import React from 'react';
import GlbViewer from './GlbViewer';

type Props = {
  caption?: string;
  height?: number;
  eager?: boolean;
};

export default function RestockingCellViewer({
  caption = 'Combined restocking cell: robot on the rail with the surveyed workcell pose.',
  height,
  eager,
}: Props) {
  return <GlbViewer kind="combined" caption={caption} height={height} eager={eager} />;
}
