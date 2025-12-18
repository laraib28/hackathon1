import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Humanoid Robotics & Physical AI',
  tagline: "A Beginner's Guide to Embodied Intelligence",
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  // Vercel URL
  url: 'https://hackathon1-9y2e.vercel.app',
  baseUrl: '/',

  organizationName: 'laraib28',
  projectName: 'humanoid-robotics-book',

  onBrokenLinks: 'warn',

  // Custom fields for API configuration
  customFields: {
    apiUrl: process.env.REACT_APP_API_URL || 'http://localhost:8000',
  },

  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
        htmlLang: 'en-US',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',
        htmlLang: 'ur-PK',
      },
    },
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
          customCss: ['./src/css/custom.css', './src/css/rtl.css'],
        },
      } satisfies Preset.Options,
    ],
  ],

  markdown: {
    mermaid: true,
  },
  themes: ['@docusaurus/theme-mermaid'],

  themeConfig: {
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
          type: 'localeDropdown',
          position: 'right',
        },
        { type: 'search', position: 'right' },
        {
          to: '/login',
          label: 'Sign In',
          position: 'right',
        },
        {
          to: '/signup',
          label: 'Sign Up',
          position: 'right',
          className: 'button button--primary button--sm',
        },
        {
          href: 'https://github.com/laraib28/humanoid-robotics-book',
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
            { label: 'Foundations', to: '/part1-foundations/intro' },
            { label: 'ROS 2 Basics', to: '/part2-modules/module1-ros2/chapter-04-introduction-to-ros2' },
            { label: 'Digital Twin', to: '/part2-modules/module2-digital-twin/chapter-08-what-is-digital-twin' },
            { label: 'Isaac Platform', to: '/part2-modules/module3-isaac/chapter-12-introduction-nvidia-isaac' },
          ],
        },
        {
          title: 'Advanced Topics',
          items: [
            { label: 'VLA Models', to: '/part2-modules/module4-vla/chapter-16-what-is-vla' },
            { label: 'System Architecture', to: '/part3-capstone/chapter-20-complete-architecture' },
            { label: 'Career Paths', to: '/part4-future/chapter-24-career-paths' },
          ],
        },
        {
          title: 'Resources',
          items: [
            { label: 'GitHub', href: 'https://github.com/laraib28/humanoid-robotics-book' },
            { label: 'ROS 2 Docs', href: 'https://docs.ros.org/en/humble/' },
            { label: 'NVIDIA Isaac', href: 'https://developer.nvidia.com/isaac-ros' },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Humanoid Robotics & Physical AI Book.`,
    },
  } satisfies Preset.ThemeConfig,
} satisfies Config;

export default config;
