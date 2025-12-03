# Contributing to Physical AI Book

Thank you for your interest in contributing to the Physical AI Book! This guide will help you understand how to create and structure content for this documentation site.

## Content Structure

All book content is stored in the `docs/` directory as Markdown files organized by chapters.

## Frontmatter Schema

Every markdown file in the `docs/` directory must include frontmatter metadata at the top of the file. Frontmatter is written in YAML format between triple dashes (`---`).

### Required Fields

All content files **must** include these fields:

```yaml
---
title: "Page Title"
description: "Brief description of the page content"
difficulty: "beginner" | "intermediate" | "advanced"
chapter: "Chapter Name"
chapter_number: 1
---
```

- **title** (string): The page title displayed in the browser and navigation
- **description** (string): A brief 1-2 sentence description of what this page covers
- **difficulty** (string): Reading level - must be one of: `beginner`, `intermediate`, or `advanced`
- **chapter** (string): The chapter name this page belongs to (e.g., "Foundations", "Mechanics")
- **chapter_number** (number): The chapter number (e.g., 1, 2, 3)

### Optional Fields

You may also include these optional fields for enhanced functionality:

```yaml
---
# ... required fields above ...
learning_objectives:
  - "Understand the fundamental concepts of Physical AI"
  - "Identify key components of robotic systems"
prerequisites:
  - "Basic understanding of programming"
  - "Familiarity with physics concepts"
estimated_time: "15 minutes"
keywords:
  - "robotics"
  - "physical AI"
  - "automation"
---
```

- **learning_objectives** (array of strings): List of specific learning goals for this page
- **prerequisites** (array of strings): List of knowledge required before reading this page
- **estimated_time** (string): Approximate reading time (e.g., "10 minutes", "30 minutes")
- **keywords** (array of strings): SEO keywords and search terms related to this content

### Example Frontmatter

Here's a complete example:

```yaml
---
title: "Introduction to Physical AI"
description: "Explore the foundational concepts of Physical AI and how it differs from traditional AI systems."
difficulty: "beginner"
chapter: "Foundations"
chapter_number: 1
learning_objectives:
  - "Define Physical AI and its key characteristics"
  - "Distinguish between Physical AI and traditional AI"
  - "Identify real-world applications of Physical AI"
prerequisites:
  - "None - this is an introductory chapter"
estimated_time: "12 minutes"
keywords:
  - "physical AI"
  - "embodied AI"
  - "robotics"
  - "artificial intelligence"
---
```

## Content Guidelines

### Markdown Features

You can use all standard Markdown features:

- **Headings**: Use `#` for headings (H1 should match the frontmatter title)
- **Lists**: Ordered and unordered lists
- **Tables**: For structured data
- **Images**: Place images in `static/img/` and reference with `![alt text](/img/filename.png)`
- **Code blocks**: Use triple backticks with language identifier for syntax highlighting

### Code Blocks

For code examples, specify the language for proper syntax highlighting:

````markdown
```python
def hello_world():
    print("Hello, Physical AI!")
```
````

Supported languages: Python, JavaScript, C++, Java, Rust, Go, TypeScript, Bash, and more.

### Progressive Complexity

Organize content to follow a progressive learning path:

1. **Beginner**: Foundational concepts, no prior knowledge assumed
2. **Intermediate**: Building on basics, assumes foundational understanding
3. **Advanced**: Deep technical details, assumes comprehensive background

## Directory Structure

Organize chapters in the `docs/` directory:

```
docs/
├── intro.md                          # Homepage introduction
├── chapter-01-foundations/           # Chapter 1
│   ├── index.md                      # Chapter overview
│   ├── 01-introduction.md            # First section
│   ├── 02-history.md                 # Second section
│   └── 03-key-concepts.md            # Third section
├── chapter-02-mechanics/             # Chapter 2
│   ├── index.md
│   └── ...
└── glossary.md                       # Optional glossary
```

## Testing Your Changes

Before submitting changes, test locally:

1. **Start development server**: `npm run start`
2. **Check rendering**: Verify your content displays correctly
3. **Check navigation**: Ensure sidebar navigation works
4. **Run build**: `npm run build` to verify no errors

## Accessibility Requirements

All content must meet WCAG 2.1 Level AA standards:

- Use descriptive link text (not "click here")
- Provide alt text for all images
- Use proper heading hierarchy (don't skip levels)
- Ensure sufficient color contrast
- Support keyboard navigation

## Pull Request Process

1. Create a feature branch from `main`
2. Add your content following the guidelines above
3. Test your changes locally
4. Submit a pull request with a clear description
5. CI/CD pipeline will run automated tests
6. Wait for review and approval

## CI/CD Pipeline

This project uses GitHub Actions for continuous integration and deployment.

### Workflow Overview

The CI/CD pipeline (`.github/workflows/deploy.yml`) runs automatically on every push and pull request to the `main` branch:

#### 1. Validate Links (`validate-links` job)
- Checks out code
- Installs dependencies and Playwright browsers
- Builds the Docusaurus site
- Starts local server for testing
- Runs link validation tests (T051)
- **Success Criteria**: 100% of internal links must resolve correctly (SC-005)

#### 2. Test Navigation (`test-navigation` job)
- Runs navigation structure tests (T021-T024)
- Tests sidebar, breadcrumbs, prev/next navigation
- Tests homepage display and content rendering
- Runs deployment smoke test (T054)
- Uploads test results artifacts on failure

#### 3. Test Accessibility (`test-accessibility` job)
- Runs axe-core accessibility tests (T052)
- Tests keyboard navigation (T053)
- Validates WCAG 2.1 AA compliance
- Tests multiple pages including search functionality
- Uploads accessibility test results on failure

#### 4. Lighthouse CI (`lighthouse` job)
- Runs Lighthouse audits for performance, accessibility, best practices, SEO
- **Success Criteria** (defined in `.lighthouserc.js`):
  - Performance: ≥85
  - Accessibility: ≥90
  - Best Practices: ≥90
  - SEO: ≥90
- Uploads Lighthouse results for review

#### 5. Build and Deploy (`deploy` job)
- **Only runs if all tests pass** (T062)
- Only deploys on pushes to `main` branch (not PRs)
- Builds the Docusaurus site (T060)
- Configures GitHub Pages
- Deploys to GitHub Pages (T061)
- Site available at: `https://sajid-khan-afridi.github.io/hackathon1/`

### Workflow Failure Handling

The workflow is designed to prevent broken deployments:

- The `deploy` job has `needs: [validate-links, test-navigation, test-accessibility, lighthouse]`
- Deployment only happens if all four test jobs complete successfully
- If any test fails, the deployment is automatically skipped
- Test results and artifacts are uploaded for debugging (retained for 7 days)

### Running CI Locally

You can run the same checks locally before pushing:

```bash
# Install dependencies
npm ci

# Install Playwright browsers
npx playwright install

# Build the site
npm run build

# Start the server in background
npm run serve &

# Wait for server to be ready
npx wait-on http://localhost:3000/hackathon1 --timeout 60000

# Run all Playwright tests
npx playwright test

# Run specific test suites
npx playwright test tests/links.test.js
npx playwright test tests/navigation.test.js
npx playwright test tests/accessibility.test.js

# Run Lighthouse CI
npm install -g @lhci/cli
lhci autorun --config=.lighthouserc.js
```

### GitHub Pages Configuration

To enable automatic deployment to GitHub Pages, configure your repository:

#### Step 1: Enable GitHub Actions Permissions

1. Go to your repository on GitHub
2. Click **Settings** → **Actions** → **General**
3. Under "Workflow permissions":
   - Select: ✅ **Read and write permissions**
   - Check: ✅ **Allow GitHub Actions to create and approve pull requests**
4. Click **Save**

#### Step 2: Configure GitHub Pages Source

1. Go to **Settings** → **Pages**
2. Under "Build and deployment":
   - **Source**: Select **GitHub Actions**
3. Click **Save**

#### Step 3: Verify Deployment

After pushing to the `main` branch:

1. Go to the **Actions** tab in your repository
2. Watch the "Deploy to GitHub Pages" workflow run
3. Once complete (✅ green), your site will be live at:
   - `https://sajid-khan-afridi.github.io/hackathon1/`

#### Step 4: Monitor Deployment Status

Monitor deployments via:

1. **Actions Tab**: View workflow runs and logs
2. **Environments**: **Settings** → **Environments** → **github-pages**
3. **Status Badge**: README badge shows current build status (once added)

### Troubleshooting CI/CD

If deployment fails:

1. **Check Workflow Logs**: Go to Actions tab → failed workflow → click on failed job
2. **Review Failed Tests**: Look for which job failed (links, navigation, accessibility, or lighthouse)
3. **Download Artifacts**: Failed test runs upload artifacts with detailed results
4. **Fix and Push**: Resolve issues and push again (workflow runs automatically)

**Common Issues**:

| Issue | Solution |
|-------|----------|
| Broken links | Check internal links in markdown files, verify URLs are correct |
| Accessibility violations | Run `npx playwright test tests/accessibility.test.js` locally, check axe-core violations |
| Performance issues | Optimize images, reduce JavaScript bundle size, check Lighthouse report |
| Build failures | Check console logs, verify all markdown files have valid frontmatter |
| Test timeouts | Increase timeout values, check if dev server is running |

### Test Coverage

The CI/CD pipeline ensures comprehensive quality:

- **SC-005**: 100% internal link validation (T051)
- **T021-T024**: Navigation structure, sidebar, breadcrumbs, homepage
- **T037-T040**: Search functionality (Phase 4)
- **T052-T053**: WCAG 2.1 AA accessibility compliance and keyboard navigation
- **T054**: Deployment smoke test
- **Lighthouse CI**: Performance, accessibility, best practices, SEO thresholds

## Questions or Issues?

If you have questions or encounter issues, please open a GitHub issue in the repository.

---

Thank you for contributing to the Physical AI Book! 🤖
