// @ts-check
const {themes: prismThemes} = require('prism-react-renderer');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Humanoid Robotics & Physical AI',
  tagline: "A Beginner's Guide to Embodied Intelligence",
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  // Correct URL for Vercel
  url: 'https://hackathon1-9y2e.vercel.app',

  // IMPORTANT: baseUrl must ALWAYS be "/"
  baseUrl: '/',

  // These do not matter for Vercel but set correctly
  organizationName: 'laraib28',
  projectName: 'humanoid-robotics-book',

  onBrokenLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.ts',
          routeBasePath: '/', // site opens directly on docs root
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  markdown: {
    mermaid: true,
  },
  themes: ['@docusaurus/theme
