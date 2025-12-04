# Physical AI Book

[![Deploy to GitHub Pages](https://github.com/sajid-khan-afridi/hackathon1/actions/workflows/deploy.yml/badge.svg)](https://github.com/sajid-khan-afridi/hackathon1/actions/workflows/deploy.yml)

A comprehensive static documentation site for the Physical AI book, built with [Docusaurus 3.x](https://docusaurus.io/).

## 📚 Read the Book

**[Start Reading →](https://sajid-khan-afridi.github.io/hackathon1/docs/intro)**

Visit the full documentation at: https://sajid-khan-afridi.github.io/hackathon1/docs/intro

## Prerequisites

- **Node.js 18+**: Required for building and running the site
- **npm**: Package manager (comes with Node.js)
- **Git**: Version control

## Installation

Install all dependencies:

```bash
npm install
```

## Local Development

Start the development server:

```bash
npm run start
```

This command starts a local development server and opens up a browser window at `http://localhost:3000`. Most changes are reflected live without having to restart the server.

## Build

Generate static files for production:

```bash
npm run build
```

This command generates static content into the `build/` directory and can be served using any static contents hosting service.

## Testing the Build Locally

After building, you can test the production build locally:

```bash
npm run serve
```

This serves the built website locally to verify everything works before deployment.

## Deployment

The site is automatically deployed to GitHub Pages when changes are pushed to the `main` branch via GitHub Actions CI/CD pipeline.

Manual deployment (if needed):

```bash
GIT_USER=<Your GitHub username> npm run deploy
```

## Project Structure

```
/
├── docs/                    # Book content (markdown chapters)
├── src/                     # Custom components and pages
├── static/                  # Static assets (images, files)
├── docusaurus.config.ts     # Site configuration
├── sidebars.ts              # Sidebar navigation structure
└── package.json             # Dependencies
```

## Contributing

See [CONTRIBUTING.md](./CONTRIBUTING.md) for content authoring guidelines, frontmatter schema, and testing requirements.

## License

Copyright © 2025 Physical AI Book. Built with Docusaurus.
