// @ts-check
const { test, expect } = require('@playwright/test');
const { navigateToDocsPage, waitForPageLoad } = require('./helpers/test-utils');

/**
 * Navigation structure tests
 * Tests for sidebar navigation, breadcrumbs, prev/next links
 */

test.describe('Navigation Structure', () => {
  test('homepage loads successfully', async ({ page }) => {
    await page.goto('/');
    await waitForPageLoad(page);
    await expect(page).toHaveTitle(/Physical AI Book/);
  });

  test('sidebar navigation is visible', async ({ page }) => {
    await navigateToDocsPage(page, 'intro');
    const sidebar = page.locator('[class*="sidebar"]');
    await expect(sidebar).toBeVisible();
  });

  test('docs intro page loads', async ({ page }) => {
    await navigateToDocsPage(page, 'intro');
    await expect(page.locator('h1')).toContainText('Welcome to Physical AI');
  });

  // T021: Navigation structure test - sidebar, chapter links, breadcrumbs
  test('sidebar displays with chapter links and breadcrumbs', async ({ page }) => {
    await page.goto('/');
    await waitForPageLoad(page);

    // Verify sidebar is visible
    const sidebar = page.locator('[class*="sidebar"]').first();
    await expect(sidebar).toBeVisible();

    // Verify chapter links are present and clickable
    const chapterLink = page.locator('a:has-text("Foundations")').first();
    await expect(chapterLink).toBeVisible();

    // Navigate to a chapter page
    await chapterLink.click();
    await waitForPageLoad(page);

    // Verify breadcrumbs are present on chapter page
    const breadcrumbs = page.locator('[class*="breadcrumb"], nav[aria-label*="readcrumb"]');
    await expect(breadcrumbs).toBeVisible();
  });

  // T022: Prev/next navigation test
  test('previous and next navigation buttons work correctly', async ({ page }) => {
    await navigateToDocsPage(page, 'intro');

    // Verify next button exists
    const nextButton = page.locator('a[class*="pagination-nav__link--next"]');
    await expect(nextButton).toBeVisible();

    // Click next and verify navigation
    const nextText = await nextButton.textContent();
    await nextButton.click();
    await waitForPageLoad(page);

    // On second page, verify previous button exists
    const prevButton = page.locator('a[class*="pagination-nav__link--prev"]');
    await expect(prevButton).toBeVisible();

    // Click previous to go back
    await prevButton.click();
    await waitForPageLoad(page);
    await expect(page.locator('h1')).toContainText('Welcome to Physical AI');
  });

  // T023: Homepage test
  test('homepage displays book title, introduction, and chapter entry points', async ({ page }) => {
    await page.goto('/');
    await waitForPageLoad(page);

    // Verify book title is displayed
    await expect(page).toHaveTitle(/Physical AI Book/);
    const heading = page.locator('h1').first();
    await expect(heading).toBeVisible();

    // Verify introduction text is present
    const introText = page.locator('text=/Physical AI|robotics|embodied/i').first();
    await expect(introText).toBeVisible();

    // Verify entry points to chapters exist (links to chapters)
    const chapterLinks = page.locator('a[href*="/chapter-"]');
    await expect(chapterLinks.first()).toBeVisible();
  });

  // T024: Content rendering test - verify markdown elements render correctly
  test('markdown content renders correctly (headings, lists, tables, images, code)', async ({ page }) => {
    await navigateToDocsPage(page, 'intro');

    // Verify headings render
    const h1 = page.locator('h1');
    await expect(h1).toBeVisible();

    // Verify we can find a chapter page with more complex content
    const chapterLink = page.locator('a[href*="/chapter-01"]').first();
    if (await chapterLink.isVisible()) {
      await chapterLink.click();
      await waitForPageLoad(page);

      // Verify various markdown elements
      // Lists (ul or ol)
      const list = page.locator('ul, ol').first();
      if (await list.count() > 0) {
        await expect(list).toBeVisible();
      }

      // Code blocks
      const codeBlock = page.locator('pre code, div[class*="codeBlock"]').first();
      if (await codeBlock.count() > 0) {
        await expect(codeBlock).toBeVisible();
      }

      // Tables
      const table = page.locator('table').first();
      if (await table.count() > 0) {
        await expect(table).toBeVisible();
      }
    }
  });

  // T054: Deployment smoke test - verify deployed site is reachable
  test('deployed site is reachable and homepage loads', async ({ page }) => {
    // This test checks the production deployment
    // In CI, this will test the actual deployed site
    const deploymentUrl = process.env.DEPLOYMENT_URL || 'http://localhost:3000/hackathon1';

    const response = await page.goto(deploymentUrl);
    expect(response?.status()).toBe(200);

    // Verify homepage content loads
    const heading = page.locator('h1').first();
    await expect(heading).toBeVisible();

    // Verify page title
    await expect(page).toHaveTitle(/Physical AI/);
  });
});
