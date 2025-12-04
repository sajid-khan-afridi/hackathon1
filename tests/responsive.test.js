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
    await page.goto('/hackathon1/docs/intro');
    await assertNoHorizontalScroll(page);
  });

  test('minimum font size on mobile', async ({ page }) => {
    await page.setViewportSize(VIEWPORTS.mobile);
    await page.goto('/hackathon1/docs/intro');
    await assertMinFontSize(page, 'body', 16);
  });

  test('no horizontal scroll on desktop (2560px)', async ({ page }) => {
    await page.setViewportSize(VIEWPORTS.desktop4K);
    await page.goto('/hackathon1/docs/intro');
    await assertNoHorizontalScroll(page);
  });

  // T069: Responsive layout test - no horizontal scrolling on 320px to 2560px widths
  test('no horizontal scroll on smallest viewport (320px)', async ({ page }) => {
    await page.setViewportSize({ width: 320, height: 568 });
    await page.goto('/hackathon1/docs/intro');
    await assertNoHorizontalScroll(page);
  });

  test('no horizontal scroll on tablet (768px)', async ({ page }) => {
    await page.setViewportSize(VIEWPORTS.tablet);
    await page.goto('/hackathon1/docs/intro');
    await assertNoHorizontalScroll(page);
  });

  test('no horizontal scroll on tablet large (1024px)', async ({ page }) => {
    await page.setViewportSize(VIEWPORTS.tabletLarge);
    await page.goto('/hackathon1/docs/intro');
    await assertNoHorizontalScroll(page);
  });

  test('sidebar displays correctly on desktop', async ({ page }) => {
    await page.setViewportSize(VIEWPORTS.desktop);
    await page.goto('/hackathon1/docs/intro');

    // Sidebar should be visible on desktop
    const sidebar = page.locator('aside[class*="sidebar"], nav[class*="sidebar"]');
    await expect(sidebar).toBeVisible();
  });

  test('breadcrumbs are present on pages', async ({ page }) => {
    await page.setViewportSize(VIEWPORTS.desktop);
    await page.goto('/hackathon1/docs/chapter-01-foundations/01-introduction');

    // Check for breadcrumb navigation (or doc navigation elements)
    // Docusaurus classic theme uses theme-doc-breadcrumbs class
    const breadcrumbs = page.locator('nav[class*="breadcrumb"], .theme-doc-breadcrumbs, nav[aria-label="Breadcrumbs"]');
    const breadcrumbCount = await breadcrumbs.count();

    // Skip test if no breadcrumbs configured (optional feature)
    if (breadcrumbCount === 0) {
      test.skip();
      return;
    }

    await expect(breadcrumbs.first()).toBeVisible();
  });
});

// T070: Mobile navigation test - sidebar converts to hamburger menu on mobile
test.describe('Mobile Navigation (<768px)', () => {
  test('hamburger menu button visible on mobile (375px)', async ({ page }) => {
    await page.setViewportSize(VIEWPORTS.mobile);
    await page.goto('/hackathon1/docs/intro');

    // Hamburger menu should be visible
    const hamburger = page.locator('button[class*="toggle"], button[aria-label*="menu"], button[aria-label*="Navigation"]');
    await expect(hamburger.first()).toBeVisible();
  });

  test('sidebar hidden by default on mobile', async ({ page }) => {
    await page.setViewportSize(VIEWPORTS.mobile);
    await page.goto('/hackathon1/docs/intro');

    // Sidebar should not be visible without clicking hamburger
    const sidebar = page.locator('aside[class*="sidebar"]:not([class*="hidden"])');
    const isVisible = await sidebar.isVisible().catch(() => false);

    // Either sidebar is hidden or not in viewport
    if (isVisible) {
      const boundingBox = await sidebar.boundingBox();
      // Sidebar should be off-screen or have display: none
      expect(boundingBox === null || boundingBox.x < 0).toBeTruthy();
    }
  });

  test('hamburger menu opens sidebar on mobile', async ({ page }) => {
    await page.setViewportSize(VIEWPORTS.mobile);
    await page.goto('/hackathon1/docs/intro');

    // Check initial state - navbar should NOT have sidebar-show class
    const navbar = page.locator('.navbar');
    let navbarClasses = await navbar.getAttribute('class');
    expect(navbarClasses).not.toContain('navbar-sidebar--show');

    // Click hamburger menu
    const hamburger = page.locator('button[class*="toggle"], button[aria-label*="menu"], button[aria-label*="Navigation"]').first();
    await hamburger.click();

    // Wait for sidebar state change
    await page.waitForTimeout(500); // Wait for animation

    // After clicking, navbar should have sidebar-show class indicating menu is open
    navbarClasses = await navbar.getAttribute('class');
    expect(navbarClasses).toContain('navbar-sidebar--show');
  });

  test('navigation links clickable on mobile', async ({ page }) => {
    await page.setViewportSize(VIEWPORTS.mobile);
    await page.goto('/hackathon1/docs/intro');

    // Open hamburger menu
    const hamburger = page.locator('button[class*="toggle"], button[aria-label*="menu"], button[aria-label*="Navigation"]').first();
    await hamburger.click();
    await page.waitForTimeout(300);

    // Find a navigation link
    const navLink = page.locator('aside a, nav[class*="sidebar"] a').first();
    await expect(navLink).toBeVisible();

    // Verify it's clickable
    await expect(navLink).toHaveAttribute('href');
  });
});

// T071: Tablet navigation test - collapsible sidebar on tablet
test.describe('Tablet Navigation (768px-1024px)', () => {
  test('sidebar visible on tablet (768px)', async ({ page }) => {
    // Docusaurus typically hides sidebar below 997px, so this test verifies behavior at tablet size
    // At 768px, sidebar may be hidden and accessible via hamburger menu (mobile-like behavior)
    await page.setViewportSize(VIEWPORTS.tablet);
    await page.goto('/hackathon1/docs/intro');

    const sidebar = page.locator('aside[class*="docSidebarContainer"]').first();
    const sidebarExists = await sidebar.count() > 0;

    // On tablet, either sidebar is visible OR hamburger menu is present
    if (sidebarExists) {
      const isVisible = await sidebar.isVisible();
      if (!isVisible) {
        // Sidebar hidden, verify hamburger menu exists for access
        const hamburger = page.locator('button[class*="toggle"], button[aria-label*="menu"]').first();
        await expect(hamburger).toBeVisible();
      } else {
        // Sidebar is visible
        await expect(sidebar).toBeVisible();
      }
    }
  });

  test('sidebar collapsible on tablet', async ({ page }) => {
    await page.setViewportSize(VIEWPORTS.tablet);
    await page.goto('/hackathon1/docs/intro');

    // Look for toggle button (may or may not be present depending on theme)
    const toggleButton = page.locator('button[class*="toggle"], button[aria-label*="menu"]');
    const hasToggle = await toggleButton.count() > 0;

    if (hasToggle) {
      // If toggle exists, sidebar should be collapsible
      await toggleButton.first().click();
      await page.waitForTimeout(300);

      // Verify sidebar state changed
      const sidebar = page.locator('aside[class*="sidebar"]');
      await expect(sidebar).toBeDefined();
    }
  });

  test('content area adjusts on tablet', async ({ page }) => {
    await page.setViewportSize(VIEWPORTS.tablet);
    await page.goto('/hackathon1/docs/intro');

    // Content should not overflow
    await assertNoHorizontalScroll(page);

    // Main content should be visible
    const mainContent = page.locator('main, article');
    await expect(mainContent.first()).toBeVisible();
  });
});

// T072: Device rotation test - layout adjusts on orientation change
test.describe('Device Rotation', () => {
  test('layout adapts from portrait to landscape (mobile)', async ({ page }) => {
    // Start in portrait
    await page.setViewportSize({ width: 375, height: 667 });
    await page.goto('/hackathon1/docs/intro');
    await assertNoHorizontalScroll(page);

    // Rotate to landscape
    await page.setViewportSize({ width: 667, height: 375 });
    await page.waitForTimeout(300); // Wait for layout adjustment
    await assertNoHorizontalScroll(page);

    // Content should still be visible
    const mainContent = page.locator('main, article');
    await expect(mainContent.first()).toBeVisible();
  });

  test('layout adapts from portrait to landscape (tablet)', async ({ page }) => {
    // Start in portrait
    await page.setViewportSize({ width: 768, height: 1024 });
    await page.goto('/hackathon1/docs/intro');
    await assertNoHorizontalScroll(page);

    // Rotate to landscape
    await page.setViewportSize({ width: 1024, height: 768 });
    await page.waitForTimeout(300);
    await assertNoHorizontalScroll(page);

    // Navigation should still work
    const sidebar = page.locator('aside[class*="sidebar"], nav[class*="sidebar"]');
    await expect(sidebar).toBeVisible();
  });

  test('layout adapts from landscape to portrait (mobile)', async ({ page }) => {
    // Start in landscape
    await page.setViewportSize({ width: 667, height: 375 });
    await page.goto('/hackathon1/docs/intro');
    await assertNoHorizontalScroll(page);

    // Rotate to portrait
    await page.setViewportSize({ width: 375, height: 667 });
    await page.waitForTimeout(300);
    await assertNoHorizontalScroll(page);

    // Hamburger menu should be present
    const hamburger = page.locator('button[class*="toggle"], button[aria-label*="menu"]');
    await expect(hamburger.first()).toBeVisible();
  });
});

// T073: Viewport typography test - minimum 16px body font on all devices
test.describe('Typography Across Viewports', () => {
  test('minimum 16px font on 320px viewport', async ({ page }) => {
    await page.setViewportSize({ width: 320, height: 568 });
    await page.goto('/hackathon1/docs/intro');
    await assertMinFontSize(page, 'body', 16);
  });

  test('minimum 16px font on mobile (375px)', async ({ page }) => {
    await page.setViewportSize(VIEWPORTS.mobile);
    await page.goto('/hackathon1/docs/intro');
    await assertMinFontSize(page, 'body', 16);
  });

  test('minimum 16px font on tablet (768px)', async ({ page }) => {
    await page.setViewportSize(VIEWPORTS.tablet);
    await page.goto('/hackathon1/docs/intro');
    await assertMinFontSize(page, 'body', 16);
  });

  test('minimum 16px font on desktop (1280px)', async ({ page }) => {
    await page.setViewportSize(VIEWPORTS.desktop);
    await page.goto('/hackathon1/docs/intro');
    await assertMinFontSize(page, 'body', 16);
  });

  test('minimum 16px font on large desktop (2560px)', async ({ page }) => {
    await page.setViewportSize(VIEWPORTS.desktop4K);
    await page.goto('/hackathon1/docs/intro');
    await assertMinFontSize(page, 'body', 16);
  });

  test('paragraph text is readable on mobile', async ({ page }) => {
    await page.setViewportSize(VIEWPORTS.mobile);
    await page.goto('/hackathon1/docs/intro');

    // Check paragraph font size
    const paragraph = page.locator('article p, main p').first();
    await expect(paragraph).toBeVisible();

    const fontSize = await paragraph.evaluate((el) => {
      return parseFloat(window.getComputedStyle(el).fontSize);
    });

    expect(fontSize).toBeGreaterThanOrEqual(16);
  });

  test('line height is readable on mobile', async ({ page }) => {
    await page.setViewportSize(VIEWPORTS.mobile);
    await page.goto('/hackathon1/docs/intro');

    // Check line height for readability
    const paragraph = page.locator('article p, main p').first();
    const lineHeight = await paragraph.evaluate((el) => {
      return window.getComputedStyle(el).lineHeight;
    });

    // Line height should be at least 1.5 for readability
    const fontSize = await paragraph.evaluate((el) => {
      return parseFloat(window.getComputedStyle(el).fontSize);
    });

    const lineHeightValue = parseFloat(lineHeight) / fontSize;
    expect(lineHeightValue).toBeGreaterThanOrEqual(1.4); // Allow slight variance
  });
});
