# Implementation Plan: Docusaurus Static Site for Physical AI Book

**Branch**: `1-docusaurus` | **Date**: 2025-12-02 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/1-docusaurus/spec.md`

## Summary

Build a Docusaurus 3.x static documentation site for the Physical AI book with automated GitHub Pages deployment. The site will provide readers with fast, accessible book content through intuitive navigation, search functionality, and responsive design. All infrastructure will use free-tier services with test-first development and WCAG 2.1 AA accessibility compliance.

---

## FIVE PLAN PILLARS

### 1️⃣ TECHNICAL APPROACH — Overall Strategy

**Core Strategy**: Leverage Docusaurus 3.x as a batteries-included static site generator to minimize custom development and maximize built-in capabilities (navigation, search, responsive design, SEO).

**Key Decisions**:

| Decision | Chosen Approach | Rationale |
|----------|----------------|-----------|
| **Static Site Generator** | Docusaurus 3.x | Constitutional mandate (Principle II); battle-tested for technical docs; includes search, navigation, responsive design out-of-box |
| **Hosting** | GitHub Pages | Free tier; native GitHub integration; HTTPS by default; constitutional requirement (Principle III) |
| **CI/CD** | GitHub Actions | Free tier; tight GitHub integration; supports test-first workflow |
| **Content Format** | Markdown with frontmatter | Standard for documentation; Docusaurus native support; enables metadata (FR-019) |
| **Search** | Docusaurus built-in search | No external dependencies; works offline; free tier |
| **Testing Strategy** | Test-first with Playwright + axe-core | Constitutional mandate (Principle VII); supports accessibility testing (FR-016) |
| **Theme** | Docusaurus classic theme (customized) | Proven accessibility baseline; responsive by default; customizable |

**Architecture Pattern**: **JAMstack (JavaScript, APIs, Markup)**
- **JavaScript**: Minimal client-side (Docusaurus runtime, search)
- **APIs**: None (static site only, no backend)
- **Markup**: Pre-rendered HTML from Markdown at build time

**Build Pipeline Flow**:
```
Author commits → GitHub Push → GitHub Actions Trigger
    ↓
Build Tests (link check, accessibility)
    ↓
Docusaurus Build (static site generation)
    ↓
Test Generated Site (navigation, search, responsive)
    ↓
Deploy to GitHub Pages (if all tests pass)
```

---

### 2️⃣ MAJOR COMPONENTS — Logical Parts

**Component Breakdown** (7 major components):

#### **Component 1: Docusaurus Core Configuration**
- **Purpose**: Configure Docusaurus runtime, site metadata, theme settings
- **Key Files**: `docusaurus.config.js`, `sidebars.js`, `package.json`
- **Responsibilities**:
  - Site metadata (title, description, URL)
  - Theme configuration (colors, logo, footer)
  - Plugin configuration (search, sitemap, SEO)
  - Build settings (base URL, organization)
- **FRs Addressed**: FR-001, FR-010, FR-011

#### **Component 2: Content Structure & Navigation**
- **Purpose**: Organize book chapters and sections with hierarchical navigation
- **Key Files**: `docs/` directory structure, `sidebars.js`, chapter markdown files with frontmatter
- **Responsibilities**:
  - Chapter organization (docs/intro/, docs/chapter-01/, etc.)
  - Sidebar navigation tree (auto-generated from directory structure)
  - Frontmatter metadata (learning objectives, difficulty, chapter info per FR-019)
  - Previous/next navigation (automatic from sidebar order)
- **FRs Addressed**: FR-002, FR-004, FR-005, FR-019

#### **Component 3: Search Functionality**
- **Purpose**: Enable full-text search across all book content
- **Key Files**: Docusaurus search plugin configuration
- **Responsibilities**:
  - Index all markdown content during build
  - Provide search UI component (search box)
  - Handle search queries with result highlighting
  - Display "no results" state
- **FRs Addressed**: FR-003

#### **Component 4: Content Rendering Engine**
- **Purpose**: Convert Markdown to HTML with code highlighting and media support
- **Key Files**: Markdown processors, Prism configuration, MDX components
- **Responsibilities**:
  - Render markdown (headings, lists, tables)
  - Syntax highlighting for code blocks (FR-008)
  - Image optimization and lazy loading
  - Responsive media embedding
- **FRs Addressed**: FR-005, FR-008, FR-009

#### **Component 5: CI/CD Pipeline**
- **Purpose**: Automated testing, building, and deployment
- **Key Files**: `.github/workflows/deploy.yml`, test scripts
- **Responsibilities**:
  - Trigger on push to main branch
  - Run link validation (FR-018)
  - Run accessibility tests with axe-core (FR-016, FR-017)
  - Run navigation/search/responsive tests (FR-015)
  - Build Docusaurus site
  - Deploy to GitHub Pages (only if all tests pass per FR-014)
  - Monitor build status and report errors
- **FRs Addressed**: FR-006, FR-013, FR-014, FR-015, FR-018

#### **Component 6: Accessibility & Responsive Design**
- **Purpose**: Ensure WCAG 2.1 AA compliance and responsive layouts
- **Key Files**: Custom CSS, theme overrides, accessibility test suite
- **Responsibilities**:
  - WCAG 2.1 AA compliance (FR-016)
  - Keyboard navigation support (FR-017)
  - Responsive breakpoints (320px-2560px)
  - Mobile-friendly navigation (collapsible sidebar)
  - Accessible color contrast and typography
- **FRs Addressed**: FR-009, FR-016, FR-017

#### **Component 7: Quality Gates & Monitoring**
- **Purpose**: Enforce quality standards before deployment
- **Key Files**: Test suites, CI configuration, Lighthouse CI
- **Responsibilities**:
  - Link validation (100% internal links valid)
  - Accessibility testing (zero axe-core violations)
  - Performance testing (Lighthouse score ≥90)
  - Build success verification
  - Deployment rollback on failure (keep previous version live)
- **FRs Addressed**: FR-012, FR-013, FR-014, FR-015, FR-018

---

### 3️⃣ DEPENDENCIES — What Blocks What

**Dependency Graph** (Sequential Dependencies):

```
Phase 0: Foundation Setup
    ├── Initialize Docusaurus project (BLOCKS all)
    ├── Configure GitHub repository (BLOCKS CI/CD)
    └── Set up development environment (BLOCKS all)
         ↓
Phase 1: Core Site & Content
    ├── Configure Docusaurus (BLOCKS content, search, theme)
    ├── Create content structure (BLOCKS navigation, search tests)
    ├── Implement navigation (BLOCKS user testing)
    └── Add sample content (BLOCKS all testing)
         ↓
Phase 2: Search & Enhancement
    ├── Configure search (REQUIRES content structure)
    ├── Add syntax highlighting (REQUIRES content rendering)
    └── Implement responsive design (REQUIRES theme configuration)
         ↓
Phase 3: CI/CD & Testing
    ├── Create GitHub Actions workflow (REQUIRES site build)
    ├── Implement link validation (REQUIRES content)
    ├── Add accessibility tests (REQUIRES site structure)
    └── Configure Lighthouse CI (REQUIRES deployment)
         ↓
Phase 4: Production Hardening
    ├── Optimize performance (REQUIRES CI/CD metrics)
    ├── Full accessibility audit (REQUIRES all features)
    └── Load testing (REQUIRES production deployment)
```

**Critical Path** (must complete in order):
1. Docusaurus initialization → 2. Content structure → 3. Navigation → 4. CI/CD → 5. Deployment

**Parallel Opportunities** (can be done concurrently within phases):
- Search configuration + Syntax highlighting (Phase 2)
- Link validation + Accessibility tests (Phase 3)
- Performance optimization + SEO tuning (Phase 4)

**External Dependencies**:
- **GitHub Pages availability**: Site deployment depends on GitHub service uptime
- **Node.js 18+**: Build environment must support Node.js 18 or higher
- **npm registry**: Package installation depends on npm availability
- **Docusaurus 3.x release**: Must use stable Docusaurus 3.x version

---

### 4️⃣ PHASES — Milestones with Deliverables

**Phase 0: Foundation Setup** (MVP Prerequisite)
- **Goal**: Initialize project infrastructure
- **Duration**: 1-2 days
- **Deliverables**:
  - ✅ Docusaurus 3.x project initialized
  - ✅ GitHub repository configured
  - ✅ Node.js 18+ environment verified
  - ✅ Basic `docusaurus.config.js` with site metadata
  - ✅ Initial `package.json` with dependencies
  - ✅ README with setup instructions
- **Blockers Removed**: Enables all subsequent development

---

**Phase 1: Core Site & Content Structure** (MVP Target)
- **Goal**: Functional static site with book content and navigation
- **Duration**: 3-5 days
- **Deliverables**:
  - ✅ Content directory structure (`docs/` with chapter organization)
  - ✅ Sidebar navigation configured (`sidebars.js`)
  - ✅ 3-5 sample chapters with markdown content
  - ✅ Frontmatter schema defined (learning objectives, difficulty, chapter info)
  - ✅ Homepage with book introduction (FR-010)
  - ✅ Previous/next navigation working (FR-004)
  - ✅ Local development server running
  - ✅ Responsive theme applied (FR-009)
- **Tests Required**:
  - Manual navigation testing (homepage → chapter in 2 clicks)
  - Responsive breakpoint testing (320px-2560px)
  - Content rendering validation (markdown features work)
- **Success Criteria**:
  - SC-007: Navigate from homepage to any chapter in 2 clicks ✅
  - Site loads locally without errors ✅
  - Content is readable on mobile and desktop ✅

---

**Phase 2: Search & Enhancement** (Beta Target)
- **Goal**: Add search functionality and enhance content rendering
- **Duration**: 2-3 days
- **Deliverables**:
  - ✅ Docusaurus search plugin configured (FR-003)
  - ✅ Search UI integrated in navigation bar
  - ✅ Syntax highlighting configured for major languages (Python, JavaScript, C++, etc.) (FR-008)
  - ✅ Code block theme customized
  - ✅ Image optimization configured
  - ✅ Responsive design tested on multiple devices
  - ✅ Mobile navigation (collapsible sidebar) working
- **Tests Required**:
  - Search query testing (various keywords)
  - Search result relevance validation
  - "No results" state testing
  - Syntax highlighting for all supported languages
  - Responsive layout on 5+ device sizes
- **Success Criteria**:
  - Search returns results in under 1 second (SC-002) ✅
  - Syntax highlighting works for all code blocks ✅
  - Site displays without horizontal scrolling (320px-2560px) (SC-004) ✅

---

**Phase 3: CI/CD & Automated Testing** (Beta Target)
- **Goal**: Implement automated deployment pipeline with quality gates
- **Duration**: 3-4 days
- **Deliverables**:
  - ✅ GitHub Actions workflow created (`.github/workflows/deploy.yml`)
  - ✅ Link validation script (checks all internal/external links) (FR-018)
  - ✅ Accessibility test suite (axe-core integration) (FR-016, FR-017)
  - ✅ Navigation structure validation tests (FR-015)
  - ✅ Search functionality tests (FR-015)
  - ✅ Responsive behavior tests (FR-015)
  - ✅ Lighthouse CI configured (performance, accessibility, SEO)
  - ✅ GitHub Pages deployment configured
  - ✅ Test-first workflow documented
- **Tests Required**:
  - Link checker finds 100% of internal links (SC-005)
  - axe-core reports zero accessibility violations (SC-008)
  - Lighthouse performance score ≥90 (SC-006)
  - All tests pass before deployment allowed (FR-014)
- **Success Criteria**:
  - Content updates live within 5 minutes (SC-003) ✅
  - 100% internal links resolve correctly (SC-005) ✅
  - Build pipeline succeeds for 99% of commits (SC-009) ✅
  - Deployment auto-rollback on test failure ✅

---

**Phase 4: Production Hardening & Optimization** (v1.0 Target)
- **Goal**: Optimize for performance, accessibility, and scale
- **Duration**: 2-3 days
- **Deliverables**:
  - ✅ Performance optimization (image compression, lazy loading, bundle splitting)
  - ✅ Full WCAG 2.1 AA accessibility audit and fixes
  - ✅ SEO optimization (meta tags, sitemap, robots.txt) (FR-011)
  - ✅ Load testing with 500+ pages
  - ✅ 3G network performance testing
  - ✅ Browser compatibility testing (Chrome, Firefox, Safari, Edge - last 2 versions)
  - ✅ Error state handling (404 page, build failure messages)
  - ✅ Production monitoring setup (build success rate tracking)
- **Tests Required**:
  - Lighthouse score ≥90 on homepage and 3 chapter pages
  - axe-core zero violations across all pages
  - Load time <3 seconds on 50Mbps connection
  - 500-page site build completes in <10 minutes
  - Site usable on 3G (1Mbps) connection
- **Success Criteria**:
  - Site loads in under 3 seconds (SC-001) ✅
  - WCAG 2.1 AA compliance with zero violations (SC-008) ✅
  - Lighthouse score ≥90 (SC-006) ✅
  - Supports 500+ pages without performance degradation (SC-010) ✅

---

### 5️⃣ SUCCESS CRITERIA — How You'll Know Each Phase is Complete

**Phase 0 Success Criteria** (Foundation):
- [ ] `npm run start` launches local development server without errors
- [ ] `npm run build` produces static site in `build/` directory
- [ ] GitHub repository has main branch with initial commit
- [ ] `docusaurus.config.js` contains correct site metadata
- [ ] Node.js version ≥18 confirmed in CI environment

**Phase 1 Success Criteria** (MVP):
- [ ] Homepage displays book title and introduction (FR-010)
- [ ] Sidebar navigation shows chapter tree (FR-002)
- [ ] Clicking sidebar items navigates to chapter pages (FR-002)
- [ ] Previous/next navigation works on all chapter pages (FR-004)
- [ ] Markdown renders correctly (headings, lists, tables, images) (FR-005)
- [ ] Frontmatter displays learning objectives (FR-019)
- [ ] Site is responsive (no horizontal scroll 320px-2560px) (SC-004)
- [ ] Can navigate homepage → any chapter in 2 clicks (SC-007) ✅

**Phase 2 Success Criteria** (Beta):
- [ ] Search box visible in navigation
- [ ] Search returns results in <1 second for 95% of queries (SC-002) ✅
- [ ] Search result clicking navigates to correct page with highlighting
- [ ] "No results" message displays for non-existent queries
- [ ] Code blocks have syntax highlighting (Python, JavaScript, C++, etc.) (FR-008)
- [ ] Mobile sidebar collapses into hamburger menu
- [ ] Tablet view shows collapsible navigation
- [ ] Device rotation adjusts layout appropriately

**Phase 3 Success Criteria** (Beta):
- [ ] GitHub Actions workflow triggers on push to main
- [ ] Link validation checks 100% of internal links (SC-005) ✅
- [ ] Link checker fails build if broken links found (FR-018)
- [ ] axe-core accessibility tests run in CI (FR-016)
- [ ] axe-core reports zero violations (SC-008) ✅
- [ ] Lighthouse CI runs on every build (SC-006)
- [ ] Lighthouse performance score ≥90 (SC-006) ✅
- [ ] Navigation, search, responsive tests pass (FR-015)
- [ ] All tests must pass before deployment (FR-014)
- [ ] Content updates appear on site within 5 minutes (SC-003) ✅
- [ ] Failed builds keep previous site version live
- [ ] Build success rate ≥99% over 30 days (SC-009) ✅

**Phase 4 Success Criteria** (v1.0):
- [ ] Site loads completely in <3 seconds on 50Mbps (SC-001) ✅
- [ ] Lighthouse performance score ≥90 on homepage + 3 chapters (SC-006) ✅
- [ ] WCAG 2.1 AA manual audit passes with zero violations (SC-008) ✅
- [ ] Full keyboard navigation works (Tab, Enter, Arrow keys) (FR-017)
- [ ] Site supports 500 pages without performance degradation (SC-010) ✅
- [ ] Build time for 500-page site <10 minutes
- [ ] Site usable on 3G (1Mbps) connection
- [ ] Browser compatibility confirmed (Chrome, Firefox, Safari, Edge - last 2 versions)
- [ ] 404 page displays for broken URLs
- [ ] Build failure messages are clear and actionable
- [ ] SEO meta tags present on all pages (FR-011)

---

## Technical Context

**Language/Version**: Node.js 18+ (LTS), JavaScript/TypeScript (Docusaurus runtime)
**Primary Dependencies**: Docusaurus 3.x, React 18, Prism (syntax highlighting), axe-core (accessibility testing)
**Storage**: Git repository (content in Markdown), Static files (GitHub Pages hosting)
**Testing**: Playwright (navigation/search tests), axe-core (accessibility), Lighthouse CI (performance), link checker
**Target Platform**: Static site hosted on GitHub Pages (HTTPS required)
**Project Type**: Web (static documentation site)
**Performance Goals**: <3s load time, <1s search response, Lighthouse ≥90, <10min build
**Constraints**: <1GB site size, <100GB/month bandwidth, <10 builds/hour (GitHub Pages limits)
**Scale/Scope**: 500 pages (50 chapters × 10 sections), WCAG 2.1 AA compliance, free tier only

---

## Constitution Check

*GATE: Must pass before Phase 0. Re-checked after Phase 3.*

| Principle | Requirement | Compliance |
|-----------|-------------|------------|
| **I. Content-First Documentation** | Markdown, frontmatter, progressive complexity | ✅ FR-019, content structure in Phase 1 |
| **II. Docusaurus Architecture** | Must use Docusaurus 3.x, docs/ structure | ✅ Technical Approach, constitutional mandate |
| **III. GitHub Pages Deployment** | Automated CI/CD, link checking, accessibility testing | ✅ FR-006, FR-013, FR-018, Phase 3 |
| **VII. Test-First for Critical Paths** | Tests before implementation, red-green-refactor | ✅ FR-013/14/15, Phase 3 test suite |

**Status**: ✅ **PASS** - All applicable constitutional principles satisfied

---

## Project Structure

### Documentation (this feature)

```text
specs/1-docusaurus/
├── spec.md              # Feature specification (completed)
├── plan.md              # This file (implementation plan)
├── checklists/
│   └── requirements.md  # Spec validation checklist
└── [Future: tasks.md created by /sp.tasks]
```

### Source Code (repository root)

```text
# Docusaurus Static Site Structure
/
├── docs/                           # Book content (markdown chapters)
│   ├── intro.md                    # Introduction/homepage content
│   ├── chapter-01/                 # Chapter 1 sections
│   │   ├── index.md                # Chapter overview
│   │   ├── section-01.md
│   │   └── section-02.md
│   ├── chapter-02/                 # Chapter 2 sections
│   └── ...                         # Additional chapters
│
├── static/                         # Static assets
│   ├── img/                        # Images, diagrams
│   └── files/                      # Downloadable resources
│
├── src/                            # Custom components (if needed)
│   ├── components/                 # Custom React components
│   ├── css/                        # Custom styles
│   └── pages/                      # Custom pages (404, etc.)
│
├── .github/                        # GitHub configuration
│   └── workflows/
│       └── deploy.yml              # CI/CD pipeline
│
├── tests/                          # Test suites
│   ├── navigation.test.js          # Navigation structure tests
│   ├── search.test.js              # Search functionality tests
│   ├── accessibility.test.js       # axe-core accessibility tests
│   ├── links.test.js               # Link validation tests
│   └── responsive.test.js          # Responsive behavior tests
│
├── docusaurus.config.js            # Main configuration
├── sidebars.js                     # Navigation structure
├── package.json                    # Dependencies
├── .lighthouserc.js                # Lighthouse CI config
└── README.md                       # Setup instructions
```

**Structure Decision**: Using Docusaurus standard structure (docs/ for content, static/ for assets) per Constitutional Principle II. This enables automatic sidebar generation, search indexing, and previous/next navigation without custom code. Tests are in separate `tests/` directory following test-first mandate (Constitutional Principle VII).

---

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**Status**: No violations. All constitutional principles satisfied by chosen approach.

---

## Edge Cases & Mitigation Strategies

**From Spec Edge Cases** (14 identified):

| Edge Case | Mitigation Strategy | Phase |
|-----------|---------------------|-------|
| Page with no content / under construction | Add custom "Under Construction" component with frontmatter flag | Phase 1 |
| Broken internal links | Automated link checker in CI (FR-018); build fails if links broken | Phase 3 |
| Build process failure | GitHub Actions error notifications; keep previous site live; clear error messages | Phase 3 |
| JavaScript disabled | Core content still accessible (static HTML); search degrades gracefully | Phase 2 |
| GitHub Pages service unavailable | Monitor status page; no mitigation possible (external dependency) | Phase 4 |
| Search with special characters | Configure search to escape/sanitize special chars; test edge cases | Phase 2 |
| Content exceeds 1GB limit | Monitor repository size; implement image optimization; warn authors at 800MB | Phase 4 |
| Very large images/videos | Configure max file size in CI; reject commits with files >5MB; docs for optimization | Phase 1 |
| Concurrent deployments | GitHub Actions queues builds automatically; no custom handling needed | Phase 3 |
| Non-English/RTL languages in search | Configure search plugin for UTF-8; test with diacritics; RTL deferred (out of scope) | Phase 2 |
| Deeply nested chapters (5+ levels) | Warn in docs; Docusaurus supports unlimited depth; test navigation UX at 5 levels | Phase 1 |
| Duplicate page titles/slugs | Docusaurus auto-generates unique slugs; add validation in CI to catch duplicates | Phase 3 |
| Deleted chapter with existing links | Link checker catches broken links (FR-018); fail build | Phase 3 |
| Slow networks (2G/3G) | Test on throttled connection; optimize images; lazy load; document min requirements | Phase 4 |

---

## Risk Register

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Docusaurus 3.x breaking changes | Low | High | Pin specific version; test upgrades in staging |
| GitHub Pages outage | Low | High | No mitigation; communicate downtime to users |
| Build time exceeds 10 minutes | Medium | Medium | Profile builds; optimize large image processing; paginate if needed |
| Accessibility violations slip through | Medium | High | Automated axe-core tests in CI; manual audits in Phase 4 |
| Search performance degrades with scale | Low | Medium | Test with 500 pages; consider external search if needed (future) |
| Browser compatibility issues | Low | Medium | Test on target browsers; use Docusaurus standard APIs |

---

## Next Steps

**Immediate Actions**:
1. ✅ Review and approve this plan
2. ⏳ Run `/sp.tasks` to generate task breakdown from this plan
3. ⏳ Begin Phase 0: Foundation Setup
4. ⏳ Set up local development environment (Node.js 18+, npm)
5. ⏳ Initialize Docusaurus project: `npx create-docusaurus@latest physical-ai-book classic`

**Questions for Stakeholders** (if any):
- None identified. Spec and plan are comprehensive. Proceed to implementation.

---

**Plan Status**: ✅ COMPLETE — Ready for `/sp.tasks` to generate task breakdown
