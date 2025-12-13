import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.glowEffect}>
          <Heading as="h1" className="hero__title neon-text">
            {siteConfig.title}
          </Heading>
        </div>
        {/* <p className="hero__subtitle neon-text-subtle">{siteConfig.tagline}</p> */}
        <p className="hero__description">
          A comprehensive guide for university students with AI/CS backgrounds learning Physical AI for humanoid robotics,
          focusing on ROS 2 as the middleware for robot control, bridging Python AI agents to ROS controllers,
          URDF for humanoids, and embodied intelligence in physical environments.
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg neon-button"
            to="/docs/intro">
            Start Learning - 4 Core Chapters ⏱️
          </Link>
        </div>

        {/* Module navigation cards */}
        {/* <div className={styles.modulesGrid}>
          <div className="card">
            <div className="card__header">
              <h3>Module 1: The Robotic Nervous System</h3>
            </div>
            <div className="card__body">
              <p>ROS 2 as the middleware for robot control, bridging Python AI agents to ROS controllers</p>
            </div>
            <div className="card__footer">
              <Link className="button button--primary" to="/docs/chapter-1-ros2-architecture">
                Explore Module 1
              </Link>
            </div>
          </div>

          <div className="card">
            <div className="card__header">
              <h3>Module 2: The Digital Twin</h3>
            </div>
            <div className="card__body">
              <p>Physics simulation, environment building, sensor simulation with Gazebo & Unity</p>
            </div>
            <div className="card__footer">
              <Link className="button button--primary" to="/docs/modules/module-02-digital-twin-sim">
                Explore Module 2
              </Link>
            </div>
          </div>

          <div className="card">
            <div className="card__header">
              <h3>Module 3: The AI-Robot Brain</h3>
            </div>
            <div className="card__body">
              <p>Photorealistic sim, synthetic data gen, Isaac ROS for VSLAM/navigation, Nav2 for bipedal movement</p>
            </div>
            <div className="card__footer">
              <Link className="button button--primary" to="/docs/modules/module-03-ai-robot-brain">
                Explore Module 3
              </Link>
            </div>
          </div>
          <div className="card">
            <div className="card__header">
              <h3>Module 4: VLA Integration</h3>
            </div>
            <div className="card__body">
              <p>VLA integration in capstone project</p>
            </div>
            <div className="card__footer">
              <Link className="button button--primary" to="/docs/modules/module-03-ai-robot-brain">
                Explore Module 3
              </Link>
            </div>
          </div>
        </div> */}
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI and Robotics`}
      description="A comprehensive guide for university students learning Physical AI for humanoid robotics, focusing on ROS 2 as the middleware for robot control, bridging Python AI agents to ROS controllers, URDF for humanoids, and embodied intelligence in physical environments.">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
