# ADR-001: Documentation Platform Stack

**Status**: Accepted
**Date**: 2025-12-02
**Deciders**: Architecture Team, Product Owner
**Technical Story**: [specs/1-docusaurus/spec.md](../../specs/1-docusaurus/spec.md)

## Context

We need to select a static site generation platform for the Physical AI book that provides fast, accessible content delivery with minimal infrastructure complexity. The solution must support:

- **Constitutional Requirements**: Must use Docusaurus 3.x (Principle II), deploy to GitHub Pages (Principle III), and use free-tier services only
- **Technical Requirements**: Static site generation from Markdown, full-text search, syntax highlighting, responsive design, automated CI/CD
- **Performance Targets**: <3s page load, <1s search response, Lighthouse score ≥90, support 500+ pages
- **Accessibility**: WCAG 2.1 Level AA compliance mandatory
- **Constraints**: <1GB site size, <100GB/month bandwidth, <10 builds/hour (GitHub Pages limits)

The platform must be battle-tested for technical documentation, provide built-in capabilities to minimize custom development, and integrate seamlessly with GitHub-based workflows.

## Decision

We will use **Docusaurus 3.x + GitHub Pages + GitHub Actions** as an integrated documentation platform stack:

### Core Components:

1. **Static Site Generator**: Docusaurus 3.x
   - React 18-based framework for content-heavy sites
   - Built-in search, navigation, responsive design, SEO
   - MDX support for interactive content
   - Proven accessibility baseline with customization options

2. **Hosting**: GitHub Pages
   - Free tier (1GB storage, 100GB/month bandwidth)
   - Native GitHub integration with HTTPS by default
   - Automatic deployment from repository
   - Reliable CDN distribution

3. **CI/CD**: GitHub Actions
   - Free tier for public repositories
   - Tight integration with GitHub Pages deployment
   - Supports test-first workflow (automated testing before deployment)
   - Build matrix for cross-platform testing

### Architecture Pattern: JAMstack
- **JavaScript**: Minimal client-side (Docusaurus runtime, search)
- **APIs**: None (static site only, no backend needed)
- **Markup**: Pre-rendered HTML from Markdown at build time

### Build Pipeline:
```
Author commits → GitHub Push → GitHub Actions
    ↓
Run Tests (link validation, accessibility, performance)
    ↓
Build Docusaurus Site (static HTML/CSS/JS generation)
    ↓
Deploy to GitHub Pages (only if all tests pass)
```

## Consequences

### Positive:

- **Zero Infrastructure Cost**: Free tier for everything (Docusaurus OSS, GitHub Pages, GitHub Actions)
- **Batteries Included**: Search, navigation, responsive design, SEO work out-of-box (minimal custom code)
- **Fast Time-to-Market**: Docusaurus scaffold + sample content → MVP in days, not weeks
- **Constitutional Compliance**: Satisfies Principles II (Docusaurus mandate) and III (GitHub Pages mandate)
- **Strong Ecosystem**: Large community, extensive plugins, proven at scale (Meta, Firebase, Redux docs use Docusaurus)
- **Performance by Default**: Static HTML = fast load times, no server-side rendering overhead
- **Git-Based Workflow**: Authors use familiar Git/Markdown workflow, no CMS needed
- **Accessibility Baseline**: Docusaurus classic theme has WCAG 2.1 AA foundation (customizable to full compliance)

### Negative:

- **Lock-In Risk**: Tightly coupled to Docusaurus conventions (docs/ structure, sidebars.js, frontmatter schema)
  - **Mitigation**: Markdown content is portable; only config files are Docusaurus-specific
- **Build Time Scaling**: Large sites (>1000 pages) may exceed 10-minute build budget
  - **Mitigation**: Profile builds at 500 pages (spec target); implement pagination/lazy loading if needed
- **Limited Dynamic Functionality**: Static site = no server-side logic, database, real-time features
  - **Mitigation**: This is a feature (simplicity), not a bug; RAG chatbot will be separate service
- **GitHub Pages Constraints**: 1GB size limit, 100GB/month bandwidth limit, 10 builds/hour limit
  - **Mitigation**: Monitor usage; implement image optimization (Phase 4); warn authors at 800MB threshold
- **Search Limitations**: Docusaurus built-in search uses client-side indexing (no advanced features like fuzzy search, filters)
  - **Mitigation**: Adequate for 500-page book; can migrate to Algolia DocSearch if needed (still free tier)
- **Version Pinning Required**: Docusaurus 3.x upgrades could introduce breaking changes
  - **Mitigation**: Pin specific version (e.g., 3.0.1); test upgrades in staging branch before production

### Risks:

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Docusaurus 3.x breaking changes | Low | High | Pin specific version; test upgrades in staging |
| GitHub Pages service outage | Low | High | No mitigation possible; communicate downtime to users; consider CDN backup for critical use |
| Build time exceeds 10 minutes at scale | Medium | Medium | Profile builds at 500 pages; optimize image processing; implement pagination if needed |

## Alternatives Considered

### Alternative 1: Next.js + Vercel + GitHub Actions

**Pros**:
- More flexible (SSR + SSG hybrid possible)
- Larger ecosystem (React-based, not docs-specific)
- Vercel provides better build performance and preview deployments

**Cons**:
- ❌ **Violates Constitution Principle II** (mandates Docusaurus 3.x)
- More configuration needed (search, navigation, responsive design not built-in)
- Steeper learning curve for content authors (Next.js conventions vs Docusaurus simplicity)
- Vercel free tier has stricter limits (100GB bandwidth/month same as GitHub Pages, but more restrictive build minutes)

**Rejected because**: Constitutional violation (non-starter)

### Alternative 2: VitePress + GitHub Pages + GitHub Actions

**Pros**:
- Faster build times (Vite vs Webpack)
- Simpler configuration (Vue-based, less abstraction)
- Lightweight runtime (smaller bundle sizes)

**Cons**:
- ❌ **Violates Constitution Principle II** (mandates Docusaurus 3.x)
- Smaller ecosystem (Vue-based, less mature for docs than Docusaurus)
- Less battle-tested for large-scale technical documentation
- Fewer plugins and integrations

**Rejected because**: Constitutional violation (non-starter)

### Alternative 3: MkDocs + Read the Docs + GitHub Actions

**Pros**:
- Python-based (familiar to some technical authors)
- Read the Docs provides free hosting with search (Elasticsearch-backed)
- Material for MkDocs theme has excellent accessibility

**Cons**:
- ❌ **Violates Constitution Principle II** (mandates Docusaurus 3.x)
- ❌ **Violates Constitution Principle III** (mandates GitHub Pages)
- Different workflow (Python vs JavaScript ecosystem)
- Search quality better than Docusaurus but Read the Docs hosting required (not GitHub Pages)

**Rejected because**: Multiple constitutional violations

### Alternative 4: Docusaurus 3.x + Cloudflare Pages + GitHub Actions

**Pros**:
- Same Docusaurus benefits
- Cloudflare Pages has better performance (faster builds, instant rollbacks)
- More generous limits (25,000 builds/month vs GitHub Pages 10/hour)

**Cons**:
- ❌ **Violates Constitution Principle III** (mandates GitHub Pages hosting)
- Introduces external dependency outside GitHub ecosystem
- Slightly more complex deployment configuration

**Rejected because**: Constitutional violation (Principle III mandates GitHub Pages)

## References

- [specs/1-docusaurus/plan.md](../../specs/1-docusaurus/plan.md) - Implementation plan with technical decisions
- [specs/1-docusaurus/spec.md](../../specs/1-docusaurus/spec.md) - Feature specification with requirements
- [.specify/memory/constitution.md](../../.specify/memory/constitution.md) - Project constitution (Principles II & III)
- [Docusaurus Documentation](https://docusaurus.io/docs) - Official framework documentation
- [GitHub Pages Documentation](https://docs.github.com/en/pages) - Hosting platform limits and capabilities

## Notes

This ADR documents the foundational platform decision that affects all subsequent technical choices. The constitutional mandates (Docusaurus + GitHub Pages) simplify the decision space but introduce constraints that must be respected. The integrated stack (Docusaurus + GitHub Pages + GitHub Actions) provides a cohesive, zero-cost solution that satisfies all requirements while maintaining simplicity.

Future decisions (e.g., search provider migration, CDN additions) should reference this ADR to maintain architectural coherence.
