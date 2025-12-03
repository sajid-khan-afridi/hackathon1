module.exports = {
  ci: {
    collect: {
      // Run Lighthouse on the production build
      staticDistDir: './build',
      // Test multiple pages
      url: [
        'http://localhost/hackathon1/docs/intro',
        'http://localhost/hackathon1/docs/chapter-01-foundations',
        'http://localhost/hackathon1/docs/chapter-02-mechanics',
      ],
      numberOfRuns: 3,
    },
    assert: {
      preset: 'lighthouse:no-pwa',
      assertions: {
        // Performance budgets (SC-006: score ≥90)
        'categories:performance': ['error', { minScore: 0.9 }],
        'categories:accessibility': ['error', { minScore: 0.9 }],
        'categories:best-practices': ['error', { minScore: 0.9 }],
        'categories:seo': ['error', { minScore: 0.9 }],

        // Specific performance metrics (SC-001: <3s load time on 50Mbps)
        'first-contentful-paint': ['error', { maxNumericValue: 2000 }],
        'largest-contentful-paint': ['error', { maxNumericValue: 3000 }],
        'cumulative-layout-shift': ['error', { maxNumericValue: 0.1 }],
        'total-blocking-time': ['error', { maxNumericValue: 300 }],

        // Accessibility requirements (SC-008: WCAG 2.1 AA)
        'color-contrast': 'error',
        'heading-order': 'error',
        'image-alt': 'error',
        'link-name': 'error',
        'meta-viewport': 'error',

        // Best practices
        'errors-in-console': 'off', // Allow console logs during development
        'valid-source-maps': 'off',
      },
    },
    upload: {
      target: 'temporary-public-storage',
    },
  },
};
