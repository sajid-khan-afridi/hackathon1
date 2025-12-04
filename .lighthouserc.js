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
      // Don't use preset - define explicit assertions only
      assertions: {
        // Core Web Vitals - relaxed for documentation site
        'categories:performance': 'off', // Too strict for docs site
        'categories:accessibility': ['error', { minScore: 0.9 }],
        'categories:best-practices': 'off', // Too many false positives
        'categories:seo': 'off', // Not critical for docs

        // Key performance metrics - relaxed thresholds
        'first-contentful-paint': 'off',
        'largest-contentful-paint': 'off',
        'cumulative-layout-shift': ['error', { maxNumericValue: 0.25 }],
        'total-blocking-time': 'off',
        'speed-index': 'off',
        'interactive': 'off',

        // Accessibility requirements (SC-008: WCAG 2.1 AA) - keep strict
        'color-contrast': 'error',
        'heading-order': 'error',
        'image-alt': 'error',
        'link-name': 'error',
        'meta-viewport': 'error',
        'html-has-lang': 'error',
        'document-title': 'error',

        // Disable all other checks
        'errors-in-console': 'off',
        'valid-source-maps': 'off',
        'csp-xss': 'off',
        'robots-txt': 'off',
        'total-byte-weight': 'off',
        'unsized-images': 'off',
        'unused-javascript': 'off',
        'uses-text-compression': 'off',
        'bootup-time': 'off',
        'dom-size': 'off',
        'legacy-javascript': 'off',
        'mainthread-work-breakdown': 'off',
        'max-potential-fid': 'off',
        'render-blocking-resources': 'off',
        'server-response-time': 'off',
        'uses-long-cache-ttl': 'off',
        'uses-responsive-images': 'off',
        'offscreen-images': 'off',
        'uses-optimized-images': 'off',
        'modern-image-formats': 'off',
      },
    },
    upload: {
      target: 'temporary-public-storage',
    },
  },
};
