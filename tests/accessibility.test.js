// @ts-check
const { test, expect } = require('@playwright/test');
const AxeBuilder = require('@axe-core/playwright').default;
const { navigateToDocsPage } = require('./helpers/test-utils');

/**
 * Accessibility tests (WCAG 2.1 AA compliance)
 * Tests using axe-core for automated accessibility checking
 */

test.describe('Accessibility', () => {
  test('homepage has no accessibility violations', async ({ page }) => {
    await page.goto('/');
    const accessibilityScanResults = await new AxeBuilder({ page }).analyze();
    expect(accessibilityScanResults.violations).toEqual([]);
  });

  test('intro page has no accessibility violations', async ({ page }) => {
    await navigateToDocsPage(page, 'intro');
    const accessibilityScanResults = await new AxeBuilder({ page }).analyze();
    expect(accessibilityScanResults.violations).toEqual([]);
  });

  // T052: Test chapter pages for accessibility violations
  test('chapter 1 page has no accessibility violations', async ({ page }) => {
    await navigateToDocsPage(page, 'chapter-01-foundations');
    const accessibilityScanResults = await new AxeBuilder({ page }).analyze();
    expect(accessibilityScanResults.violations).toEqual([]);
  });

  test('chapter 2 page has no accessibility violations', async ({ page }) => {
    await navigateToDocsPage(page, 'chapter-02-mechanics');
    const accessibilityScanResults = await new AxeBuilder({ page }).analyze();
    expect(accessibilityScanResults.violations).toEqual([]);
  });

  test('search page has no accessibility violations', async ({ page }) => {
    await page.goto('/docs/intro');

    // Open search if it exists
    const searchButton = page.locator('button[class*="DocSearch"], button[aria-label*="Search"]').first();
    if (await searchButton.isVisible({ timeout: 1000 }).catch(() => false)) {
      await searchButton.click();
      await page.waitForTimeout(500);
    }

    const accessibilityScanResults = await new AxeBuilder({ page }).analyze();
    expect(accessibilityScanResults.violations).toEqual([]);
  });

  // T053: Keyboard navigation test
  test('keyboard navigation works correctly', async ({ page }) => {
    await page.goto('/docs/intro');

    // Test Tab navigation
    await page.keyboard.press('Tab');
    const firstFocusedElement = await page.evaluate(() => document.activeElement?.tagName);
    expect(['A', 'BUTTON', 'INPUT']).toContain(firstFocusedElement);

    // Test that focused elements are visible
    const focusedElement = page.locator(':focus');
    await expect(focusedElement).toBeVisible();

    // Test Enter key activation on links
    await page.keyboard.press('Tab');
    await page.keyboard.press('Tab');
    const linkElement = page.locator('a:focus');
    if (await linkElement.count() > 0) {
      // If focused on a link, pressing Enter should navigate
      const href = await linkElement.getAttribute('href');
      if (href && href.startsWith('/docs')) {
        await page.keyboard.press('Enter');
        await page.waitForLoadState('networkidle');
        expect(page.url()).toContain('/docs');
      }
    }
  });
});
