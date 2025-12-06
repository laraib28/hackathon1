import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  emoji: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Hands-On Learning',
    emoji: '🤖',
    description: (
      <>
        Build real humanoid robot systems from scratch using ROS 2, Isaac Sim, and modern AI frameworks.
        Complete with code examples, simulations, and practical projects.
      </>
    ),
  },
  {
    title: 'Industry-Ready Skills',
    emoji: '⚡',
    description: (
      <>
        Master the complete stack: perception (Isaac ROS), planning (Nav2, MoveIt2),
        and execution (VLA models). Learn the same tools used at Boston Dynamics, Tesla, and NVIDIA.
      </>
    ),
  },
  {
    title: 'From Zero to Hero',
    emoji: '🚀',
    description: (
      <>
        No robotics background required. Progress from foundational concepts to building
        autonomous humanoid systems. Includes career paths and learning roadmaps.
      </>
    ),
  },
  {
    title: 'Modern AI Integration',
    emoji: '🧠',
    description: (
      <>
        Integrate GPT-4, Whisper, and Vision-Language-Action models for natural interaction.
        Learn to build robots that understand voice commands and reason about tasks.
      </>
    ),
  },
  {
    title: 'Simulation-First',
    emoji: '🎮',
    description: (
      <>
        Train and test in photorealistic simulation (Gazebo, Isaac Sim) before hardware deployment.
        Generate synthetic data and master sim-to-real transfer techniques.
      </>
    ),
  },
  {
    title: 'Complete Architecture',
    emoji: '🏗️',
    description: (
      <>
        Understand the 6-layer architecture from hardware control to natural language interfaces.
        Learn integration patterns, testing strategies, and production deployment.
      </>
    ),
  },
];

function Feature({title, emoji, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')} style={{marginBottom: '2rem'}}>
      <div className="features-card">
        <div className="text--center" style={{fontSize: '3rem', marginBottom: '1rem'}}>
          {emoji}
        </div>
        <div className="text--center">
          <Heading as="h3" style={{marginBottom: '1rem'}}>{title}</Heading>
          <p style={{fontSize: '0.95rem', lineHeight: '1.6', opacity: 0.9}}>{description}</p>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features} style={{padding: '4rem 0'}}>
      <div className="container">
        <div className="text--center" style={{marginBottom: '3rem'}}>
          <Heading as="h2" style={{fontSize: '2.5rem', marginBottom: '1rem'}}>
            What You'll Learn
          </Heading>
          <p style={{fontSize: '1.25rem', opacity: 0.8, maxWidth: '800px', margin: '0 auto'}}>
            A comprehensive guide to building autonomous humanoid robots with Physical AI
          </p>
        </div>
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
