// @ts-check
const { test, expect } = require('@playwright/test');

/**
 * Link validation tests
 * Tests for broken internal/external links
 * Target: 100% internal links resolve correctly (SC-005)
 */

test.describe('Link Validation', () => {
  // T051: Verify all internal links resolve correctly (100% pass rate per SC-005)
  test('all internal links resolve correctly', async ({ page }) => {
    const pagesToTest = [
      '/hackathon1/docs/intro',
      '/hackathon1/docs/chapter-01-foundations',
      '/hackathon1/docs/chapter-01-foundations/introduction',
      '/hackathon1/docs/chapter-02-mechanics',
    ];

    const brokenLinks = [];

    for (const pagePath of pagesToTest) {
      await page.goto(pagePath);

      // Find all internal links (href starts with / or #)
      const internalLinks = await page.locator('a[href^="/"], a[href^="#"]').all();

      for (const link of internalLinks) {
        const href = await link.getAttribute('href');
        if (!href || href === '#') continue; // Skip empty or fragment-only links

        // For hash links, just verify the element exists on current page
        if (href.startsWith('#')) {
          const targetId = href.substring(1);
          const targetElement = page.locator(`[id="${targetId}"]`);
          const exists = await targetElement.count() > 0;
          if (!exists) {
            brokenLinks.push({ page: pagePath, link: href, type: 'hash' });
          }
          continue;
        }

        // For absolute internal links, navigate and check status
        try {
          const response = await page.request.get(href);
          if (!response.ok()) {
            brokenLinks.push({ page: pagePath, link: href, status: response.status() });
          }
        } catch (error) {
          brokenLinks.push({ page: pagePath, link: href, error: error.message });
        }
      }
    }

    // Assert no broken links found (100% pass rate)
    expect(brokenLinks).toEqual([]);
  });
});
