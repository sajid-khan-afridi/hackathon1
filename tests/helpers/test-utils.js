// @ts-check
const { expect } = require('@playwright/test');

/**
 * Test utilities for Physical AI Book Playwright tests
 * Provides common helpers for page loading, assertions, and viewport configurations
 */

/**
 * Common viewport configurations for responsive testing
 */
const VIEWPORTS = {
  mobile: { width: 375, height: 667 },    // iPhone SE
  mobileLarge: { width: 428, height: 926 }, // iPhone 14 Pro Max
  tablet: { width: 768, height: 1024 },   // iPad Mini
  tabletLarge: { width: 1024, height: 1366 }, // iPad Pro
  desktop: { width: 1280, height: 720 },  // Standard desktop
  desktopLarge: { width: 1920, height: 1080 }, // Full HD
  desktop4K: { width: 2560, height: 1440 }, // 2K/QHD
};

/**
 * Wait for page to be fully loaded with no network activity
 * @param {import('@playwright/test').Page} page
 * @param {number} timeout - Optional timeout in milliseconds
 */
async function waitForPageLoad(page, timeout = 30000) {
  // Wait for DOM to be loaded first (more reliable)
  await page.waitForLoadState('domcontentloaded', { timeout });

  // Wait for client-side hydration to complete (Docusaurus-specific)
  // Check if the page has been hydrated by looking for the data attribute
  try {
    await page.waitForSelector('[data-has-hydrated="true"]', { timeout: 15000 });
  } catch {
    // If hydration marker not found, wait a bit for JS to execute
    await page.waitForTimeout(2000);
  }

  // In CI, skip networkidle wait as it can be unreliable with some sites
  // Locally, wait for networkidle for more thorough testing
  if (!process.env.CI) {
    try {
      await page.waitForLoadState('networkidle', { timeout: 10000 });
    } catch {
      // Ignore networkidle timeout - not critical
    }
  }
}

/**
 * Navigate to a docs page and wait for it to load
 * @param {import('@playwright/test').Page} page
 * @param {string} path - Path relative to /docs/ (e.g., 'intro', 'chapter-01-foundations')
 */
async function navigateToDocsPage(page, path) {
  // Construct full URL path for proper navigation
  // Note: baseURL in config is http://localhost:3000/hackathon1
  const url = `/hackathon1/docs/${path}`;
  await page.goto(url);
  await waitForPageLoad(page);
}

/**
 * Check if an element is visible on the page
 * @param {import('@playwright/test').Page} page
 * @param {string} selector
 * @returns {Promise<boolean>}
 */
async function isVisible(page, selector) {
  try {
    const element = await page.locator(selector);
    return await element.isVisible();
  } catch {
    return false;
  }
}

/**
 * Assert that no horizontal scrolling is present
 * @param {import('@playwright/test').Page} page
 */
async function assertNoHorizontalScroll(page) {
  const scrollWidth = await page.evaluate(() => document.documentElement.scrollWidth);
  const clientWidth = await page.evaluate(() => document.documentElement.clientWidth);
  expect(scrollWidth).toBeLessThanOrEqual(clientWidth);
}

/**
 * Get computed font size of an element
 * @param {import('@playwright/test').Page} page
 * @param {string} selector
 * @returns {Promise<number>} Font size in pixels
 */
async function getFontSize(page, selector) {
  return await page.locator(selector).evaluate((el) => {
    return parseFloat(window.getComputedStyle(el).fontSize);
  });
}

/**
 * Assert minimum font size for readability
 * @param {import('@playwright/test').Page} page
 * @param {string} selector
 * @param {number} minSize - Minimum font size in pixels (default: 16)
 */
async function assertMinFontSize(page, selector, minSize = 16) {
  const fontSize = await getFontSize(page, selector);
  expect(fontSize).toBeGreaterThanOrEqual(minSize);
}

/**
 * Check if page loads within time budget
 * @param {import('@playwright/test').Page} page
 * @param {string} url
 * @param {number} maxTime - Maximum load time in milliseconds
 */
async function assertPageLoadTime(page, url, maxTime = 3000) {
  const startTime = Date.now();
  await page.goto(url);
  await waitForPageLoad(page);
  const loadTime = Date.now() - startTime;
  expect(loadTime).toBeLessThan(maxTime);
}

/**
 * Wait for search results to appear
 * @param {import('@playwright/test').Page} page
 * @param {number} timeout - Optional timeout in milliseconds
 */
async function waitForSearchResults(page, timeout = 5000) {
  await page.waitForSelector('[class*="search"]', { timeout });
}

/**
 * Perform a search query
 * @param {import('@playwright/test').Page} page
 * @param {string} query - Search query text
 */
async function performSearch(page, query) {
  // Click search button/input
  const searchButton = page.locator('button[class*="search"], input[class*="search"]').first();
  await searchButton.click();

  // Type query
  const searchInput = page.locator('input[type="search"], input[placeholder*="Search"]').first();
  await searchInput.fill(query);
  await searchInput.press('Enter');

  // Wait for results
  await waitForSearchResults(page);
}

/**
 * Get all headings on the page in order
 * @param {import('@playwright/test').Page} page
 * @returns {Promise<Array<{level: number, text: string}>>}
 */
async function getHeadings(page) {
  return await page.evaluate(() => {
    const headings = Array.from(document.querySelectorAll('h1, h2, h3, h4, h5, h6'));
    return headings.map(h => ({
      level: parseInt(h.tagName.substring(1)),
      text: h.textContent?.trim() || '',
    }));
  });
}

/**
 * Assert proper heading hierarchy (no skipped levels)
 * @param {import('@playwright/test').Page} page
 */
async function assertHeadingHierarchy(page) {
  const headings = await getHeadings(page);

  for (let i = 1; i < headings.length; i++) {
    const prev = headings[i - 1].level;
    const curr = headings[i].level;

    // Current heading should not skip levels (e.g., h2 to h4)
    if (curr > prev) {
      expect(curr - prev).toBeLessThanOrEqual(1);
    }
  }
}

module.exports = {
  VIEWPORTS,
  waitForPageLoad,
  navigateToDocsPage,
  isVisible,
  assertNoHorizontalScroll,
  getFontSize,
  assertMinFontSize,
  assertPageLoadTime,
  waitForSearchResults,
  performSearch,
  getHeadings,
  assertHeadingHierarchy,
};
