import type {ReactNode} from 'react';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './styles.module.css';

export default function HomepageDemo(): ReactNode {
  const poster = useBaseUrl('/media/restocking-demo-poster.png');
  const src = useBaseUrl('/media/restocking-demo.mp4');
  return (
    <section className={styles.demo}>
      <div className="container">
        <figure className={styles.figure}>
          <video
            className={styles.video}
            controls
            playsInline
            preload="metadata"
            poster={poster}
            aria-label="Restocking demo recorded in the simulated workcell">
            <source src={src} type="video/mp4" />
          </video>
          <figcaption className={styles.caption}>
            A restocking run in the simulated workcell (no hardware involved). See the{' '}
            <Link to="/docs">introduction</Link> for details.
          </figcaption>
        </figure>
      </div>
    </section>
  );
}
