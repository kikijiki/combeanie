import type {ReactNode} from 'react';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './styles.module.css';

type Props = {
  /** File stem under static/media: `restocking-showcase`. */
  stem: string;
  /** Accessible name of the video. */
  label: string;
  /** One line: what the footage shows and which evidence path it uses. */
  caption: ReactNode;
};

/** One recorded demo: the MP4 with its poster, and a caption. */
export default function DemoVideo({stem, label, caption}: Props): ReactNode {
  const poster = useBaseUrl(`/media/${stem}-poster.png`);
  const src = useBaseUrl(`/media/${stem}.mp4`);
  return (
    <figure className={styles.figure}>
      <video
        className={styles.video}
        controls
        playsInline
        preload="metadata"
        poster={poster}
        aria-label={label}>
        <source src={src} type="video/mp4" />
      </video>
      <figcaption className={styles.caption}>{caption}</figcaption>
    </figure>
  );
}
