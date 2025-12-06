import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Humanoid Robotics & Physical AI',
  tagline: 'A Beginner\'s Guide to Embodied Intelligence',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://your-username.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/humanoid-robotics-book/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'your-username', // Usually your GitHub org/user name.
  projectName: 'humanoid-robotics-book', // Usually your repo name.

  onBrokenLinks: 'warn',  // Changed from 'throw' to allow build with broken links

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          routeBasePath: '/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  markdown: {
    mermaid: true,
  },
  themes: ['@docusaurus/theme-mermaid'],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Humanoid Robotics & Physical AI',
      logo: {
        alt: 'Robotics Book Logo',
        src: 'img/logo.svg',
      },
      hideOnScroll: false,
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'bookSidebar',
          position: 'left',
          label: '📚 Book',
        },
        {
          to: '/part4-future/chapter-24-career-paths',
          label: '💼 Careers',
          position: 'left',
        },
        {
          type: 'search',
          position: 'right',
        },
        {
          to: '/login',
          label: 'Sign In',
          position: 'right',
          className: 'navbar-login-link',
        },
        {
          to: '/signup',
          label: 'Sign Up',
          position: 'right',
          className: 'button button--primary button--sm',
        },
        {
          href: 'https://github.com/your-username/humanoid-robotics-book',
          position: 'right',
          className: 'header-github-link',
          'aria-label': 'GitHub repository',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Book Content',
          items: [
            {
              label: 'Foundations',
              to: '/part1-foundations/intro',
            },
            {
              label: 'ROS 2 Basics',
              to: '/module1-ros2/chapter-04-ros2-basics',
            },
            {
              label: 'Digital Twin',
              to: '/module2-digital-twin/chapter-08-gazebo-basics',
            },
            {
              label: 'Isaac Platform',
              to: '/module3-isaac/chapter-12-introduction-nvidia-isaac',
            },
          ],
        },
        {
          title: 'Advanced Topics',
          items: [
            {
              label: 'VLA Models',
              to: '/module4-vla/chapter-16-introduction-to-vla',
            },
            {
              label: 'System Architecture',
              to: '/part3-capstone/chapter-20-complete-architecture',
            },
            {
              label: 'Career Paths',
              to: '/part4-future/chapter-24-career-paths',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/your-username/humanoid-robotics-book',
            },
            {
              label: 'ROS 2 Docs',
              href: 'https://docs.ros.org/en/humble/',
            },
            {
              label: 'NVIDIA Isaac',
              href: 'https://developer.nvidia.com/isaac-ros',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Report Issues',
              href: 'https://github.com/your-username/humanoid-robotics-book/issues',
            },
            {
              label: 'Contribute',
              href: 'https://github.com/your-username/humanoid-robotics-book/blob/main/CONTRIBUTING.md',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Humanoid Robotics & Physical AI Book. Open source project for learning embodied AI. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['bash', 'python', 'yaml', 'json', 'cpp', 'cmake'],
    },
    metadata: [
      {name: 'keywords', content: 'robotics, humanoid robots, ROS 2, physical AI, embodied AI, Isaac Sim, VLA models, robot programming'},
      {name: 'author', content: 'Humanoid Robotics Book Project'},
      {property: 'og:type', content: 'website'},
    ],
    announcementBar: {
      id: 'new_book',
      content: '🎉 New comprehensive guide to humanoid robotics! <a target="_blank" rel="noopener noreferrer" href="/part1-foundations/intro">Start reading now</a>',
      backgroundColor: '#0066cc',
      textColor: '#ffffff',
      isCloseable: true,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
