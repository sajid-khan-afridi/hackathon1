# ADR-002: Testing and Quality Strategy

**Status**: Accepted
**Date**: 2025-12-02
**Deciders**: Architecture Team, QA Lead
**Technical Story**: [specs/1-docusaurus/spec.md](../../specs/1-docusaurus/spec.md)

## Context

The project constitution mandates **Test-First Development** for critical paths (Principle VII: non-negotiable). We need a testing strategy that:

- **Constitutional Requirement**: Tests written BEFORE implementation (red-green-refactor cycle mandatory)
- **Quality Gates**: Link validation, accessibility testing, performance testing, build verification
- **Automation**: All tests must run in CI; deployment blocked if any test fails (FR-014)
- **Accessibility Compliance**: WCAG 2.1 Level AA with zero violations required (FR-016, SC-008)
- **Performance Targets**: Lighthouse score ≥90, <3s load time, <1s search response
- **Coverage Requirements**: Navigation structure, search functionality, responsive behavior across devices

The testing strategy must support continuous deployment while maintaining high quality standards and constitutional compliance.

## Decision

We will implement a **Test-First Quality Strategy** using Playwright + axe-core + Lighthouse CI:

### Testing Tools & Frameworks:

1. **End-to-End Testing**: Playwright
   - Cross-browser testing (Chromium, Firefox, WebKit)
   - Navigation structure validation (FR-015)
   - Search functionality testing (FR-015)
   - Responsive behavior testing across viewports (FR-015)
   - Runs headless in CI for fast feedback

2. **Accessibility Testing**: axe-core (via @axe-core/playwright)
   - Automated WCAG 2.1 Level AA validation (FR-016, SC-008)
   - Tests homepage, chapter pages, search functionality
   - Zero violations required to pass (fail-fast on accessibility issues)
   - Keyboard navigation validation (FR-017)

3. **Performance Testing**: Lighthouse CI
   - Performance, accessibility, best practices, SEO scores
   - Lighthouse score ≥90 required for deployment (SC-006)
   - Tests homepage + 3 representative chapter pages
   - Budget enforcement (max bundle size, max load time)

4. **Link Validation**: Custom script (markdown-link-check or similar)
   - 100% internal link resolution required (SC-005, FR-018)
   - External link validation with timeout/retry
   - Build fails if any 404 errors detected

### Test-First Workflow (Red-Green-Refactor):

```
Phase 1-3 Development Cycle:
1. RED: Write failing test for new feature
   - Navigation test expects sidebar, but no sidebar exists yet
   - Test runs → FAIL (expected)

2. GREEN: Implement minimal code to pass test
   - Configure sidebars.js with chapter structure
   - Test runs → PASS

3. REFACTOR: Clean up code while maintaining passing tests
   - Optimize sidebar rendering, add error handling
   - Tests still PASS

4. COMMIT: Only commit when all tests pass
```

### CI/CD Quality Gates:

```yaml
GitHub Actions Workflow (.github/workflows/deploy.yml):

1. Checkout code
2. Install dependencies (npm ci)
3. Run linting (eslint, prettier)
4. Run unit tests (if any)
5. Build Docusaurus site (npm run build)
6. Run link validation (check all internal/external links)
7. Run accessibility tests (axe-core on built site)
8. Run navigation/search/responsive tests (Playwright)
9. Run Lighthouse CI (performance + accessibility scores)
10. IF all tests PASS → Deploy to GitHub Pages
    IF any test FAILS → Block deployment, keep previous version live
```

### Test Organization:

```
tests/
├── navigation.test.js      # Sidebar, prev/next, homepage navigation
├── search.test.js          # Search queries, results, highlighting
├── accessibility.test.js   # axe-core WCAG 2.1 AA tests
├── links.test.js           # Link validation (internal + external)
├── responsive.test.js      # Viewport testing (320px-2560px)
└── helpers/
    ├── test-utils.js       # Shared utilities (page load, assertions)
    └── fixtures.js         # Test data (sample content, URLs)
```

## Consequences

### Positive:

- **Constitutional Compliance**: Test-first mandate (Principle VII) enforced by workflow documentation and PR reviews
- **Quality Assurance**: Multiple layers of testing (functional, accessibility, performance) catch issues before production
- **Fast Feedback**: Automated tests run on every commit; developers know within minutes if changes break quality gates
- **Accessibility Guarantee**: axe-core zero violations + manual audits ensure WCAG 2.1 AA compliance (SC-008)
- **Performance Guarantee**: Lighthouse CI enforces performance budgets (SC-006); prevents performance regressions
- **Deployment Safety**: Failed tests block deployment (FR-014); previous version stays live if build/tests fail
- **Cross-Browser Coverage**: Playwright tests Chromium, Firefox, WebKit (covers Chrome, Edge, Safari, Firefox)
- **Regression Prevention**: Existing tests prevent breaking changes; refactoring is safe

### Negative:

- **Upfront Test Writing Effort**: Test-first requires writing tests before features (slower initial development)
  - **Mitigation**: Tests pay off long-term (catch bugs early, enable confident refactoring); constitutionally mandated
- **CI Build Time Increase**: Running full test suite adds 3-5 minutes to CI pipeline
  - **Mitigation**: Acceptable for quality assurance; parallelizable tests reduce wait time
- **Test Maintenance Overhead**: Tests must be updated when features change (double the code to maintain)
  - **Mitigation**: Keep tests simple and focused; use Page Object Model to reduce duplication
- **False Positives**: Lighthouse CI scores can fluctuate due to CI environment variability
  - **Mitigation**: Use median of 3 runs; allow ±5 point variance; re-run flaky tests
- **Limited axe-core Coverage**: Automated accessibility testing catches ~50% of WCAG issues (rest need manual audit)
  - **Mitigation**: Phase 4 includes full manual WCAG 2.1 AA audit to catch remaining issues

### Risks:

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Accessibility violations slip through automated tests | Medium | High | Manual WCAG audit in Phase 4; train team on accessibility best practices |
| Lighthouse CI performance variance | Medium | Low | Use median scores; allow ±5 point buffer; re-run flaky tests |
| Test suite becomes slow (>10 minutes) | Medium | Medium | Parallelize tests; run critical tests first; skip non-critical tests in draft PRs |
| Developers skip test-first workflow | High | High | PR reviews enforce test presence; CI blocks merges without tests; team training |

## Alternatives Considered

### Alternative 1: Test-After Development (Write Tests After Features)

**Pros**:
- Faster initial development (no upfront test writing)
- More flexible (can adapt tests to final implementation)

**Cons**:
- ❌ **Violates Constitution Principle VII** (test-first is non-negotiable)
- Lower test coverage (tests often skipped when under time pressure)
- Tests may be biased by implementation (less effective at catching bugs)

**Rejected because**: Constitutional violation (non-starter)

### Alternative 2: Cypress + Pa11y + Lighthouse CI

**Pros**:
- Cypress has better debugging UI (time-travel debugging)
- Pa11y is a dedicated accessibility testing tool (similar to axe-core)

**Cons**:
- Cypress has limitations on cross-browser testing (WebKit support experimental)
- Pa11y and axe-core have similar capabilities (no significant advantage)
- Playwright is faster in CI (headless mode optimized)
- Playwright has better TypeScript support (if we migrate to TS)

**Rejected because**: Playwright provides better cross-browser coverage and performance

### Alternative 3: Jest + Testing Library + Manual Accessibility Audits

**Pros**:
- Jest is familiar to React developers (component-level testing)
- Testing Library promotes accessible queries (good practice)
- Manual audits catch 100% of WCAG issues (vs 50% automated)

**Cons**:
- Component-level tests don't validate integration (navigation, search, deployment)
- Manual audits are slow and expensive (not scalable for continuous deployment)
- Combination approach still needed (automated + manual)

**Rejected because**: Inadequate for end-to-end quality gates; manual-only audits don't scale

### Alternative 4: No Automated Testing (Manual QA Only)

**Pros**:
- Zero upfront tooling investment
- Maximum flexibility (no test maintenance)

**Cons**:
- ❌ **Violates Constitution Principle VII** (test-first mandate)
- ❌ **Violates FR-014** (automated tests required before deployment)
- Manual QA is slow and error-prone (doesn't scale)
- No regression prevention (changes can break existing features)

**Rejected because**: Multiple constitutional and spec violations

## References

- [specs/1-docusaurus/plan.md](../../specs/1-docusaurus/plan.md) - Testing strategy in Phase 3
- [specs/1-docusaurus/spec.md](../../specs/1-docusaurus/spec.md) - FR-013, FR-014, FR-015, FR-016, FR-017, FR-018
- [.specify/memory/constitution.md](../../.specify/memory/constitution.md) - Principle VII (Test-First mandate)
- [Playwright Documentation](https://playwright.dev/) - E2E testing framework
- [axe-core Documentation](https://github.com/dequelabs/axe-core) - Accessibility testing engine
- [Lighthouse CI Documentation](https://github.com/GoogleChrome/lighthouse-ci) - Performance testing

## Notes

This ADR documents the testing approach that ensures constitutional compliance (Principle VII) while meeting all quality requirements. The test-first workflow is non-negotiable per the constitution, so alternatives that don't support it are excluded from consideration.

The three-layer testing strategy (functional + accessibility + performance) provides comprehensive coverage while remaining automatable in CI. Manual accessibility audits in Phase 4 supplement automated testing to achieve full WCAG 2.1 AA compliance.

Future test additions (e.g., visual regression testing, load testing) should follow the same test-first principle and integrate into the CI quality gates defined here.
