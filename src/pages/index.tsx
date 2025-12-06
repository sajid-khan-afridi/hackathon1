import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Reading - 5min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className={clsx('col col--4')}>
            <div className="text--center padding-horiz--md">
              <Heading as="h2">🤖 ROS 2 Fundamentals</Heading>
              <p>
                Learn the fundamental concepts of Physical AI, ROS 2 middleware,
                sensor systems, and robot programming with Python.
              </p>
              <Link to="/docs/module-01-ros2">
                Explore Module 1 →
              </Link>
            </div>
          </div>
          <div className={clsx('col col--4')}>
            <div className="text--center padding-horiz--md">
              <Heading as="h2">⚙️ Robot Simulation</Heading>
              <p>
                Master Gazebo and Unity simulation environments for testing
                humanoid robots with realistic physics and sensors.
              </p>
              <Link to="/docs/module-02-gazebo-unity">
                Explore Module 2 →
              </Link>
            </div>
          </div>
          <div className={clsx('col col--4')}>
            <div className="text--center padding-horiz--md">
              <Heading as="h2">📚 Comprehensive Guide</Heading>
              <p>
                From beginner to advanced topics, this book provides progressive
                complexity with examples and real-world applications.
              </p>
              <Link to="/docs/intro">
                Get Started →
              </Link>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="A comprehensive guide to Physical AI, robotics, and intelligent physical systems">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
