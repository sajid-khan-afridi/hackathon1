// @ts-check
const { test, expect } = require('@playwright/test');
const { VIEWPORTS, assertNoHorizontalScroll, assertMinFontSize } = require('./helpers/test-utils');

/**
 * Responsive design tests
 * Tests for layout behavior across different screen sizes
 */

test.describe('Responsive Design', () => {
  test('no horizontal scroll on mobile (375px)', async ({ page }) => {
    await page.setViewportSize(VIEWPORTS.mobile);
    await page.goto('/docs/intro');
    await assertNoHorizontalScroll(page);
  });

  test('minimum font size on mobile', async ({ page }) => {
    await page.setViewportSize(VIEWPORTS.mobile);
    await page.goto('/docs/intro');
    await assertMinFontSize(page, 'body', 16);
  });

  test('no horizontal scroll on desktop (2560px)', async ({ page }) => {
    await page.setViewportSize(VIEWPORTS.desktop4K);
    await page.goto('/docs/intro');
    await assertNoHorizontalScroll(page);
  });

  // Additional responsive tests will be added in Phase 6 (User Story 4)
});
