import type {ReactNode} from 'react';
import DemoVideo from '../DemoVideo';
import styles from './styles.module.css';

export default function HomepageDemo(): ReactNode {
  return (
    <section className={styles.demo}>
      <div className="container">
        <DemoVideo
          stem="restocking-showcase"
          label="Showcase restocking run in the simulated workcell"
          caption={
            <>
              <strong>Showcase scene.</strong> Labelled cans and bottles in five SKUs: a stocked
              shelf, a packed back room, and the arm moving stock between them. Ground-truth
              evidence path (<code>just demo</code>).
            </>
          }
        />
      </div>
    </section>
  );
}
