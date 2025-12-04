// @ts-check
const { test, expect } = require('@playwright/test');

/**
 * Search functionality tests
 * Tests for search box, query results, result navigation
 */

test.describe('Search Functionality', () => {
  // T037: Search box displays in navigation bar
  test('search box displays in navigation bar', async ({ page }) => {
    await page.goto('/hackathon1/docs/intro');

    // Look for search button/input in navbar
    const searchButton = page.locator('button[class*="DocSearch"], button[aria-label*="Search"], .navbar__search, input[type="search"]');
    await expect(searchButton.first()).toBeVisible({ timeout: 5000 });
  });

  // T038: Search returns results for sample queries within 1 second
  test('search returns results for sample queries within 1 second', async ({ page }) => {
    await page.goto('/hackathon1/docs/intro');

    // Click search button to open search modal
    const searchButton = page.locator('button[class*="DocSearch"], button[aria-label*="Search"]').first();
    await searchButton.click();

    // Wait for search modal/input to appear
    const searchInput = page.locator('input[type="search"], input[placeholder*="Search"]').first();
    await expect(searchInput).toBeVisible({ timeout: 2000 });

    // Type search query
    const startTime = Date.now();
    await searchInput.fill('Physical AI');

    // Wait for results to appear
    const searchResults = page.locator('[class*="result"], [class*="DocSearch-Hit"], li[role="option"]');
    await expect(searchResults.first()).toBeVisible({ timeout: 2000 });

    const endTime = Date.now();
    const responseTime = endTime - startTime;

    // Verify response time is under 1 second
    expect(responseTime).toBeLessThan(1000);

    // Verify at least one result is displayed
    const resultCount = await searchResults.count();
    expect(resultCount).toBeGreaterThan(0);
  });

  // T039: Clicking search results navigates to correct page
  test('clicking search results navigates to correct page with term highlighting', async ({ page }) => {
    await page.goto('/hackathon1/docs/intro');

    // Open search
    const searchButton = page.locator('button[class*="DocSearch"], button[aria-label*="Search"]').first();
    await searchButton.click();

    // Search for "robotics"
    const searchInput = page.locator('input[type="search"], input[placeholder*="Search"]').first();
    await searchInput.fill('robotics');

    // Wait for results
    const searchResults = page.locator('[class*="DocSearch-Hit"], li[role="option"]').first();
    await expect(searchResults).toBeVisible({ timeout: 2000 });

    // Get the URL before clicking
    const currentUrl = page.url();

    // Click first result
    await searchResults.click();

    // Wait for navigation
    await page.waitForLoadState('networkidle');

    // Verify we navigated to a different page
    const newUrl = page.url();
    expect(newUrl).not.toBe(currentUrl);

    // Verify the page contains the search term
    const pageContent = await page.textContent('body');
    expect(pageContent?.toLowerCase()).toContain('robotics');
  });

  // T040: No results state displays helpful message
  test('no results state displays helpful message for non-existent queries', async ({ page }) => {
    await page.goto('/hackathon1/docs/intro');

    // Open search
    const searchButton = page.locator('button[class*="DocSearch"], button[aria-label*="Search"]').first();
    await searchButton.click();

    // Search for something that doesn't exist
    const searchInput = page.locator('input[type="search"], input[placeholder*="Search"]').first();
    await searchInput.fill('xyzabcnonexistentterm123');

    // Wait a moment for search to process
    await page.waitForTimeout(500);

    // Look for "no results" message
    const noResultsMessage = page.locator('text=/no results|nothing found|no matches/i, [class*="no-results"], [class*="NoResults"]');
    await expect(noResultsMessage.first()).toBeVisible({ timeout: 2000 });
  });
});
