import { themes as prismThemes } from "prism-react-renderer";

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: "Angler",
  tagline:
    "A ROS 2 framework for development and deployment of lightweight underwater vehicle manipulator systems",
  favicon: "img/favicon.ico",

  url: "https://robotic-decision-making-lab.github.io",
  baseUrl: "/angler",

  organizationName: "Robotic-Decision-Making-Lab",
  projectName: "angler",

  onBrokenLinks: "throw",
  onBrokenMarkdownLinks: "warn",
  trailingSlash: true,

  i18n: {
    defaultLocale: "en",
    locales: ["en"],
  },

  presets: [
    [
      "classic",
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: "./sidebars.js",
          routeBasePath: "/",
          editUrl: "https://github.com/Robotic-Decision-Making-Lab/angler/docs",
        },
        blog: false,
        theme: {
          customCss: "./src/css/custom.scss",
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: "img/docusaurus-social-card.jpg",
      navbar: {
        title: "Angler Documentation",
        items: [
          {
            href: "https://research.engr.oregonstate.edu/rdml/home",
            label: "RDML",
            position: "right",
          },
          {
            href: "https://github.com/Robotic-Decision-Making-Lab/angler",
            position: "right",
            className: "header-github-link",
            "aria-label": "GitHub repository",
          },
        ],
      },
      footer: {
        style: "dark",
        copyright: `© ${new Date().getFullYear()} Robotic Decision Making Lab @ Oregon State University.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        additionalLanguages: ["bash"],
      },
    }),
  markdown: {
    mermaid: true,
  },
  plugins: ["docusaurus-plugin-sass", "@docusaurus/theme-mermaid"],
};

export default config;
