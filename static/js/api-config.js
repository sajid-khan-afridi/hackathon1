/**
 * API Configuration
 *
 * This file sets the API base URL for the frontend to communicate with the backend.
 * It's loaded as a script tag before the main application runs, making the
 * configuration available via window.API_CONFIG.
 *
 * Environment-based configuration:
 * - Development: http://localhost:8000
 * - Production: https://your-production-api.com
 */

(function () {
  // Detect environment (this is a simple check; adjust as needed)
  const isDevelopment =
    window.location.hostname === 'localhost' ||
    window.location.hostname === '127.0.0.1';

  // Set API base URL
  const apiUrl = isDevelopment
    ? 'http://localhost:8000'
    : 'https://your-production-api.com'; // TODO: Replace with actual production API URL

  // Make configuration available globally
  window.API_CONFIG = {
    baseUrl: apiUrl,
    timeout: 30000, // 30 seconds
  };

  console.log('[API Config] Environment:', isDevelopment ? 'development' : 'production');
  console.log('[API Config] Base URL:', apiUrl);
})();
