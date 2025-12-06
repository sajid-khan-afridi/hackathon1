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
  // PRODUCTION: Update this URL after deploying your backend
  // Options:
  //   - Render: https://physical-ai-backend.onrender.com
  //   - Railway: https://physical-ai-backend.up.railway.app
  //   - Fly.io: https://physical-ai-backend.fly.dev
  const apiUrl = isDevelopment
    ? 'http://localhost:8000'
    : 'https://physical-ai-backend.onrender.com'; // TODO: Replace with YOUR actual backend URL

  // Make configuration available globally
  window.API_CONFIG = {
    baseUrl: apiUrl,
    timeout: 30000, // 30 seconds
  };

  console.log('[API Config] Environment:', isDevelopment ? 'development' : 'production');
  console.log('[API Config] Base URL:', apiUrl);
})();
