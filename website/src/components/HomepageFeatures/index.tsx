import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  href: string;
  Icon: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

function RunIcon(props: React.ComponentProps<'svg'>) {
  return (
    <svg viewBox="0 0 64 64" fill="none" stroke="currentColor" strokeWidth="2" {...props}>
      <rect x="10" y="12" width="44" height="40" rx="2" />
      <path d="M22 28l16 8-16 8V28z" strokeLinejoin="round" />
    </svg>
  );
}

function FundamentalsIcon(props: React.ComponentProps<'svg'>) {
  return (
    <svg viewBox="0 0 64 64" fill="none" stroke="currentColor" strokeWidth="2" {...props}>
      <circle cx="20" cy="20" r="8" />
      <circle cx="44" cy="20" r="8" />
      <circle cx="32" cy="44" r="8" />
      <path d="M26 24l4 14M38 24l-4 14M28 20h8" strokeLinecap="round" />
    </svg>
  );
}

function ArchitectureIcon(props: React.ComponentProps<'svg'>) {
  return (
    <svg viewBox="0 0 64 64" fill="none" stroke="currentColor" strokeWidth="2" {...props}>
      <rect x="8" y="10" width="48" height="12" rx="2" />
      <rect x="8" y="26" width="48" height="12" rx="2" />
      <rect x="8" y="42" width="48" height="12" rx="2" />
      <path d="M32 22v4M32 38v4" strokeLinecap="round" />
    </svg>
  );
}

const FeatureList: FeatureItem[] = [
  {
    title: 'Introduction',
    href: '/docs',
    Icon: FundamentalsIcon,
    description: (
      <>
        The retail restocking problem, the baseline scenario, the robot and workcell, with
        schematics and 3D models inline.
      </>
    ),
  },
  {
    title: 'How to run',
    href: '/docs/run',
    Icon: RunIcon,
    description: (
      <>
        Set up the Nix environment, launch simulation and baseline compositions, drive demos,
        and run acceptance tests or benchmarks.
      </>
    ),
  },
  {
    title: 'Architecture',
    href: '/docs/architecture',
    Icon: ArchitectureIcon,
    description: (
      <>
        Control path, ports, and trust boundaries after you know what is being controlled.
        Fundamentals covers ROS, Gazebo, and MoveIt if you need them.
      </>
    ),
  },
];

function Feature({title, href, Icon, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Icon className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">
          <Link to={href}>{title}</Link>
        </Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props) => (
            <Feature key={props.title} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
