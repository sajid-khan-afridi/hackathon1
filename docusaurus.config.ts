import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI Book',
  tagline: 'A comprehensive guide to Physical AI and robotics',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://sajid-khan-afridi.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/hackathon1/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'sajid-khan-afridi', // Usually your GitHub org/user name.
  projectName: 'hackathon1', // Usually your repo name.

  onBrokenLinks: 'throw',

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
          routeBasePath: 'docs',
        },
        blog: false, // Disable blog functionality (out of scope)
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  // Local search plugin configuration (Phase 4 - User Story 2)
  themes: [
    [
      require.resolve("@easyops-cn/docusaurus-search-local"),
      {
        hashed: true,
        language: ["en"],
        highlightSearchTermsOnTargetPage: true,
        explicitSearchResultPath: true,
      },
    ],
  ],

  themeConfig: {
    // SEO: Social card for Open Graph and Twitter Card metadata (T096)
    image: 'img/docusaurus-social-card.jpg',
    metadata: [
      // Open Graph metadata
      {name: 'og:title', content: 'Physical AI Book - Comprehensive Guide to Robotics and Physical AI'},
      {name: 'og:description', content: 'Learn Physical AI, robotics foundations, mechanics, and cutting-edge applications. A comprehensive educational resource for students and professionals.'},
      {name: 'og:type', content: 'website'},
      {name: 'og:url', content: 'https://sajid-khan-afridi.github.io/hackathon1/'},
      {name: 'og:image', content: 'https://sajid-khan-afridi.github.io/hackathon1/img/docusaurus-social-card.jpg'},
      // Twitter Card metadata
      {name: 'twitter:card', content: 'summary_large_image'},
      {name: 'twitter:title', content: 'Physical AI Book - Comprehensive Guide to Robotics'},
      {name: 'twitter:description', content: 'Learn Physical AI, robotics foundations, mechanics, and cutting-edge applications.'},
      {name: 'twitter:image', content: 'https://sajid-khan-afridi.github.io/hackathon1/img/docusaurus-social-card.jpg'},
      // Additional SEO metadata
      {name: 'keywords', content: 'physical AI, robotics, artificial intelligence, machine learning, robotics engineering, automation'},
      {name: 'author', content: 'Physical AI Book'},
      {name: 'robots', content: 'index, follow'},
    ],
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI Book',
      logo: {
        alt: 'Physical AI Book Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Book',
        },
        {
          href: 'https://github.com/sajid-khan-afridi/hackathon1',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Book',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'GitHub Repository',
              href: 'https://github.com/sajid-khan-afridi/hackathon1',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Book. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'java', 'cpp', 'rust', 'go', 'bash', 'typescript', 'json', 'yaml'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
