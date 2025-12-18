import type {ReactNode} from 'react';
import {useState} from 'react';
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
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div style={{marginTop: '2rem', marginBottom: '1rem'}}>
          <p style={{fontSize: '1.1rem', maxWidth: '700px', margin: '0 auto 2rem', opacity: 0.9}}>
            Master the complete stack for building autonomous humanoid robots — from ROS 2 foundations
            to Vision-Language-Action models. Build production-ready systems with the same tools used at
            leading robotics companies.
          </p>
        </div>
        <div className={styles.buttons} style={{display: 'flex', gap: '1rem', justifyContent: 'center', flexWrap: 'wrap'}}>
          <Link
            className="button button--primary button--lg"
            to="/part1-foundations/intro"
            style={{minWidth: '200px'}}>
            📚 Start Reading
          </Link>
          <Link
            className="button button--secondary button--lg"
            to="/part4-future/chapter-24-career-paths"
            style={{minWidth: '200px'}}>
            💼 Career Paths
          </Link>
        </div>
        <div style={{marginTop: '2rem', display: 'flex', gap: '2rem', justifyContent: 'center', flexWrap: 'wrap', fontSize: '0.9rem', opacity: 0.8}}>
          <div>✨ 24 Comprehensive Chapters</div>
          <div>🔧 Hands-On Projects</div>
          <div>🎯 Industry-Ready Skills</div>
        </div>
      </div>
    </header>
  );
}

function BookStats() {
  const stats = [
    { value: '4', label: 'Core Modules', description: 'ROS 2 • Digital Twin • Isaac • VLA' },
    { value: '24', label: 'Chapters', description: 'Foundations to Advanced Architecture' },
    { value: '100%', label: 'Hands-On', description: 'Code Examples & Simulations' },
    { value: 'Free', label: 'Open Access', description: 'Learn at Your Own Pace' },
  ];

  return (
    <section style={{padding: '4rem 0', background: 'var(--ifm-background-color)'}}>
      <div className="container">
        <div className="row">
          {stats.map((stat, idx) => (
            <div key={idx} className="col col--3" style={{textAlign: 'center', marginBottom: '2rem'}}>
              <div style={{fontSize: '3rem', fontWeight: 'bold', color: 'var(--ifm-color-primary)', marginBottom: '0.5rem'}}>
                {stat.value}
              </div>
              <div style={{fontSize: '1.25rem', fontWeight: '600', marginBottom: '0.5rem'}}>
                {stat.label}
              </div>
              <div style={{fontSize: '0.9rem', opacity: 0.8}}>
                {stat.description}
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function BookModules() {
  const modules = [
    {
      title: 'Part 1: Foundations',
      chapters: '3 chapters',
      topics: ['Physical AI Concepts', 'Embodied Intelligence', 'Robot Hardware Basics'],
      icon: '📖'
    },
    {
      title: 'Module 1: ROS 2',
      chapters: '4 chapters',
      topics: ['Pub-Sub Architecture', 'Launch Systems', 'TF2 Transforms'],
      icon: '⚙️'
    },
    {
      title: 'Module 2: Digital Twin',
      chapters: '4 chapters',
      topics: ['Gazebo Simulation', 'URDF Models', 'Unity Integration'],
      icon: '🎮'
    },
    {
      title: 'Module 3: Isaac',
      chapters: '4 chapters',
      topics: ['Isaac ROS', 'Synthetic Data', 'Domain Randomization'],
      icon: '🔬'
    },
    {
      title: 'Module 4: VLA',
      chapters: '4 chapters',
      topics: ['LLM Planning', 'Whisper ASR', 'End-to-End Policies'],
      icon: '🧠'
    },
    {
      title: 'Part 3-4: Integration',
      chapters: '5 chapters',
      topics: ['System Architecture', 'Career Paths', 'Future Trends'],
      icon: '🚀'
    },
  ];

  return (
    <section style={{padding: '4rem 0'}}>
      <div className="container">
        <div className="text--center" style={{marginBottom: '3rem'}}>
          <Heading as="h2" style={{fontSize: '2.5rem', marginBottom: '1rem'}}>
            Complete Curriculum
          </Heading>
          <p style={{fontSize: '1.25rem', opacity: 0.8}}>
            Structured learning path from beginner to expert
          </p>
        </div>
        <div className="row">
          {modules.map((module, idx) => (
            <div key={idx} className="col col--4" style={{marginBottom: '2rem'}}>
              <div className="features-card" style={{height: '100%'}}>
                <div style={{fontSize: '2.5rem', marginBottom: '1rem'}}>
                  {module.icon}
                </div>
                <Heading as="h3" style={{fontSize: '1.5rem', marginBottom: '0.5rem'}}>
                  {module.title}
                </Heading>
                <div style={{fontSize: '0.9rem', opacity: 0.7, marginBottom: '1rem'}}>
                  {module.chapters}
                </div>
                <ul style={{fontSize: '0.95rem', lineHeight: '1.8', paddingLeft: '1.5rem'}}>
                  {module.topics.map((topic, topicIdx) => (
                    <li key={topicIdx}>{topic}</li>
                  ))}
                </ul>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function HomepageChatbot() {
  const [question, setQuestion] = useState('');
  const [answer, setAnswer] = useState('');
  const [loading, setLoading] = useState(false);

  const handleAsk = async () => {
    if (!question.trim()) return;

    setLoading(true);
    setAnswer('');

    try {
      const response = await fetch('https://hackathon1-production-aaf0.up.railway.app/api/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: question,
          language: 'en'
        }),
      });

      const data = await response.json();
      setAnswer(data.response || 'No response received');
    } catch (error) {
      setAnswer('Error: Could not connect to chatbot');
    } finally {
      setLoading(false);
    }
  };

  return (
    <section style={{padding: '4rem 0', background: 'var(--ifm-background-color)'}}>
      <div className="container">
        <div className="text--center" style={{marginBottom: '2rem'}}>
          <Heading as="h2" style={{fontSize: '2.5rem', marginBottom: '1rem'}}>
            Ask the Robotics Assistant
          </Heading>
          <p style={{fontSize: '1.1rem', opacity: 0.8}}>
            Get instant answers about humanoid robotics, ROS 2, and more
          </p>
        </div>

        <div style={{maxWidth: '800px', margin: '0 auto'}}>
          <div style={{display: 'flex', gap: '1rem', marginBottom: '2rem'}}>
            <input
              type="text"
              value={question}
              onChange={(e) => setQuestion(e.target.value)}
              onKeyPress={(e) => e.key === 'Enter' && handleAsk()}
              placeholder="Ask a question about robotics..."
              style={{
                flex: 1,
                padding: '0.75rem 1rem',
                fontSize: '1rem',
                borderRadius: '8px',
                border: '2px solid var(--ifm-color-emphasis-300)',
                outline: 'none'
              }}
            />
            <button
              onClick={handleAsk}
              disabled={loading}
              className="button button--primary button--lg"
              style={{minWidth: '120px'}}>
              {loading ? 'Asking...' : 'Ask'}
            </button>
          </div>

          {answer && (
            <div style={{
              padding: '1.5rem',
              borderRadius: '8px',
              background: 'var(--ifm-card-background-color)',
              border: '1px solid var(--ifm-color-emphasis-200)',
              lineHeight: '1.6'
            }}>
              <strong style={{display: 'block', marginBottom: '0.5rem', color: 'var(--ifm-color-primary)'}}>
                Answer:
              </strong>
              {answer}
            </div>
          )}
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={siteConfig.title}
      description="A comprehensive guide to building autonomous humanoid robots with Physical AI. Learn ROS 2, Digital Twin simulation, NVIDIA Isaac, and Vision-Language-Action models.">
      <HomepageHeader />
      <main>
        <BookStats />
        <HomepageFeatures />
        <BookModules />
        <HomepageChatbot />
      </main>
    </Layout>
  );
}
