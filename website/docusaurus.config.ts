import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'combeanie',
  tagline:
    'Vision-guided beverage restocking in simulation: ROS 2, MoveIt, and deterministic authority.',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  // Configure these at publication time; neither value establishes a deployed site.
  url: process.env.DOCS_SITE_URL ?? 'https://kikijiki.github.io',
  baseUrl: process.env.DOCS_BASE_URL ?? '/',
  trailingSlash: false,

  organizationName: 'kikijiki',
  projectName: 'combeanie',

  onBrokenLinks: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  markdown: {
    format: 'detect',
    hooks: {
      onBrokenMarkdownLinks: 'throw',
    },
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl: 'https://github.com/kikijiki/combeanie/tree/master/website/',
        },
        // User-facing manual only.
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themes: [
    [
      require.resolve('@easyops-cn/docusaurus-search-local'),
      {
        hashed: true,
        indexDocs: true,
        indexBlog: false,
        docsRouteBasePath: '/docs',
        highlightSearchTermsOnTargetPage: true,
        explicitSearchResultPath: true,
      },
    ],
  ],

  plugins: [
    [
      'docusaurus-plugin-llms',
      {
        generateLLMsTxt: true,
        generateLLMsFullTxt: true,
        docsDir: 'docs',
        includeBlog: false,
        title: 'combeanie',
        description:
          'A simulation-first vision-guided robotic beverage-restocking platform built on ROS 2, Gazebo, MoveIt 2, and deterministic world-state authority.',
      },
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'combeanie',
      logo: {
        alt: 'combeanie logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'docs',
          position: 'left',
          label: 'Docs',
        },
        {
          type: 'doc',
          docId: 'run/index',
          position: 'left',
          label: 'How to run',
        },
        {
          type: 'doc',
          docId: 'architecture/index',
          position: 'left',
          label: 'Architecture',
        },
        {
          href: 'https://github.com/kikijiki/combeanie',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            {label: 'Introduction', to: '/docs'},
            {label: 'How to run', to: '/docs/run'},
            {label: 'Fundamentals', to: '/docs/fundamentals'},
            {label: 'Architecture', to: '/docs/architecture'},
            {label: 'Glossary', to: '/docs/architecture/glossary'},
          ],
        },
        {
          title: 'Project',
          items: [
            {label: 'GitHub', href: 'https://github.com/kikijiki/combeanie'},
          ],
        },
      ],
      copyright: `Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['bash', 'python', 'yaml', 'cpp', 'cmake', 'nix', 'json'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
