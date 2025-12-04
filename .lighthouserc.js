module.exports = {
  ci: {
    collect: {
      // Test multiple pages served by npm run serve
      url: [
        'http://localhost:3000/hackathon1/docs/intro',
        'http://localhost:3000/hackathon1/docs/chapter-01-foundations',
        'http://localhost:3000/hackathon1/docs/chapter-02-mechanics',
      ],
      numberOfRuns: 3,
    },
    assert: {
      preset: 'lighthouse:no-pwa',
      assertions: {
        // Relaxed performance budgets for documentation site
        'categories:performance': ['warn', { minScore: 0.7 }],
        'categories:accessibility': ['error', { minScore: 0.9 }],
        'categories:best-practices': ['warn', { minScore: 0.8 }],
        'categories:seo': ['warn', { minScore: 0.8 }],

        // Relaxed performance metrics for documentation site
        'first-contentful-paint': ['warn', { maxNumericValue: 3000 }],
        'largest-contentful-paint': ['warn', { maxNumericValue: 5000 }],
        'cumulative-layout-shift': ['error', { maxNumericValue: 0.1 }],
        'total-blocking-time': ['warn', { maxNumericValue: 600 }],

        // Accessibility requirements (SC-008: WCAG 2.1 AA) - keep strict
        'color-contrast': 'error',
        'heading-order': 'error',
        'image-alt': 'error',
        'link-name': 'error',
        'meta-viewport': 'error',

        // Best practices - relax non-critical checks
        'errors-in-console': 'off',
        'valid-source-maps': 'off',
        'csp-xss': 'off', // CSP not critical for static docs
        'robots-txt': 'off', // Not blocking for docs site
        'total-byte-weight': 'off', // Documentation sites need more content
        'unsized-images': 'warn', // Warn but don't fail
        'unused-javascript': 'off', // Docusaurus includes framework code
        'uses-text-compression': 'off', // Server configuration, not in our control
      },
    },
    upload: {
      target: 'temporary-public-storage',
    },
  },
};
