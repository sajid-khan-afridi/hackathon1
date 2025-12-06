// @ts-check
const { defineConfig, devices } = require('@playwright/test');

/**
 * Playwright configuration for Physical AI Book
 * Tests navigation, search, accessibility, and responsive behavior
 * @see https://playwright.dev/docs/test-configuration
 */
module.exports = defineConfig({
  testDir: './tests',
  // Maximum time one test can run for
  // Increased from 30s to 60s to handle slower CI environments (T066-T067 fixes)
  timeout: process.env.CI ? 60 * 1000 : 30 * 1000,

  // Run tests in files in parallel
  fullyParallel: true,

  // Fail the build on CI if you accidentally left test.only in the source code
  forbidOnly: !!process.env.CI,

  // Retry on CI only
  retries: process.env.CI ? 2 : 0,

  // Opt out of parallel tests on CI
  workers: process.env.CI ? 1 : undefined,

  // Reporter to use
  reporter: 'html',

  // Shared settings for all the projects below
  use: {
    // Base URL to use in actions like `await page.goto('/')`
    baseURL: 'http://localhost:3000/hackathon1',

    // Increase navigation timeout for CI environments
    navigationTimeout: process.env.CI ? 30 * 1000 : 15 * 1000,

    // Increase action timeout for CI environments
    actionTimeout: process.env.CI ? 15 * 1000 : 10 * 1000,

    // Collect trace when retrying the failed test
    trace: 'on-first-retry',

    // Screenshot on failure
    screenshot: 'only-on-failure',
  },

  // Configure projects for major browsers
  // In CI, only test chromium for speed; locally test all browsers
  projects: process.env.CI ? [
    {
      name: 'chromium',
      use: { ...devices['Desktop Chrome'] },
    },
  ] : [
    {
      name: 'chromium',
      use: { ...devices['Desktop Chrome'] },
    },

    {
      name: 'firefox',
      use: {
        ...devices['Desktop Firefox'],
        // Firefox needs more time for client-side hydration
        navigationTimeout: 45 * 1000,
        actionTimeout: 20 * 1000,
      },
      timeout: 90 * 1000, // Increased timeout for Firefox tests
    },

    {
      name: 'webkit',
      use: {
        ...devices['Desktop Safari'],
        // WebKit needs more time for client-side hydration
        navigationTimeout: 45 * 1000,
        actionTimeout: 20 * 1000,
      },
      timeout: 90 * 1000, // Increased timeout for WebKit tests
    },

    // Test against mobile viewports
    // Mobile browsers need longer timeouts due to slower rendering
    {
      name: 'Mobile Chrome',
      use: { ...devices['Pixel 5'] },
      timeout: 120 * 1000, // 2 minutes for mobile
    },

    {
      name: 'Mobile Safari',
      use: { ...devices['iPhone 12'] },
      timeout: 120 * 1000, // 2 minutes for mobile
    },

    // Test against tablet viewports
    {
      name: 'iPad',
      use: { ...devices['iPad Pro'] },
    },
  ],

  // Run your local dev server before starting the tests
  // In CI, webServer is disabled to avoid port conflicts in parallel jobs
  webServer: process.env.CI ? undefined : {
    command: 'npm run start',
    url: 'http://localhost:3000/hackathon1',
    reuseExistingServer: true,
    timeout: 120 * 1000,
  },
});
