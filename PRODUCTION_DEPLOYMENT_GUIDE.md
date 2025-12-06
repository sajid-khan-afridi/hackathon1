# Production Deployment Guide

**Project**: Physical AI RAG Chatbot
**Date**: 2025-12-06
**Status**: Production Ready

---

## Architecture Overview

### Deployment Strategy

**Frontend (Docusaurus)**: GitHub Pages
- URL: `https://sajid-khan-afridi.github.io/hackathon1/`
- Static site hosting (HTML, CSS, JavaScript)
- Free, automatic SSL, CDN distribution

**Backend (FastAPI)**: Render/Railway/Fly.io
- Recommended: **Render** (easiest setup, free tier available)
- Alternative 1: **Railway** (developer-friendly)
- Alternative 2: **Fly.io** (global edge deployment)

**Why This Split?**
GitHub Pages only hosts static files. Python applications need a server environment with:
- Python runtime
- Database connections (PostgreSQL)
- Vector database access (Qdrant)
- Environment variables for secrets

---

## Option 1: Deploy Backend to Render (Recommended)

### Why Render?
- ✅ Free tier available (750 hours/month)
- ✅ Auto-deploy from GitHub
- ✅ Built-in PostgreSQL (Neon alternative)
- ✅ Environment variables management
- ✅ Automatic HTTPS
- ✅ Simple setup

### Step-by-Step: Render Deployment

#### 1. Create Render Account
1. Go to [render.com](https://render.com)
2. Sign up with GitHub account
3. Authorize Render to access your repositories

#### 2. Create PostgreSQL Database (Optional)
If not using Neon:
1. Dashboard → New → PostgreSQL
2. Name: `physical-ai-db`
3. Database: `physical_ai`
4. User: (auto-generated)
5. Region: Choose closest to users
6. Plan: Free
7. Create Database
8. **Copy Internal Database URL** (for backend connection)

#### 3. Create Web Service
1. Dashboard → New → Web Service
2. Connect Repository: `sajid-khan-afridi/hackathon1`
3. Configure:
   - **Name**: `physical-ai-backend`
   - **Region**: Same as database
   - **Branch**: `main`
   - **Root Directory**: `backend`
   - **Runtime**: `Python 3`
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`
   - **Plan**: Free

#### 4. Set Environment Variables
In Render dashboard → Environment:

```bash
# Database (use Render PostgreSQL internal URL or Neon URL)
DATABASE_URL=postgresql://user:password@hostname:5432/physical_ai

# Qdrant (use Qdrant Cloud free tier)
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-api-key

# OpenAI
OPENAI_API_KEY=sk-...

# CORS (allow your GitHub Pages domain)
ALLOWED_ORIGINS=https://sajid-khan-afridi.github.io,http://localhost:3000

# App Config
ENVIRONMENT=production
DEBUG=false
```

#### 5. Deploy
1. Click "Create Web Service"
2. Render automatically builds and deploys
3. Wait for "Live" status (2-5 minutes)
4. Copy service URL: `https://physical-ai-backend.onrender.com`

#### 6. Test Backend
```bash
curl https://physical-ai-backend.onrender.com/api/v1/health
```

Expected response:
```json
{
  "status": "healthy",
  "services": {
    "database": "healthy",
    "qdrant": "healthy",
    "openai": "healthy"
  },
  "version": "0.1.0"
}
```

---

## Option 2: Deploy Backend to Railway

### Why Railway?
- ✅ $5 free credit monthly
- ✅ Excellent developer experience
- ✅ Simple environment management
- ✅ Built-in PostgreSQL
- ✅ Auto-deploy from GitHub

### Step-by-Step: Railway Deployment

#### 1. Create Railway Account
1. Go to [railway.app](https://railway.app)
2. Sign up with GitHub
3. Authorize Railway

#### 2. Create New Project
1. Dashboard → New Project
2. Deploy from GitHub Repo
3. Select `sajid-khan-afridi/hackathon1`
4. Railway auto-detects Python

#### 3. Configure Service
1. Settings → Root Directory: `backend`
2. Settings → Start Command: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`
3. Settings → Build Command: `pip install -r requirements.txt`

#### 4. Add PostgreSQL (Optional)
1. New → Database → PostgreSQL
2. Railway auto-provisions database
3. Connection string auto-added to environment

#### 5. Set Environment Variables
Settings → Variables:

```bash
DATABASE_URL=${{Postgres.DATABASE_URL}}  # Auto-injected
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-api-key
OPENAI_API_KEY=sk-...
ALLOWED_ORIGINS=https://sajid-khan-afridi.github.io,http://localhost:3000
ENVIRONMENT=production
DEBUG=false
```

#### 6. Generate Domain
1. Settings → Networking → Generate Domain
2. Copy URL: `https://physical-ai-backend.up.railway.app`

---

## Option 3: Deploy Backend to Fly.io

### Why Fly.io?
- ✅ Free tier (3 VMs)
- ✅ Global edge deployment
- ✅ Fast performance
- ✅ Docker-based (more control)

### Step-by-Step: Fly.io Deployment

#### 1. Install Fly CLI
```bash
# Windows (PowerShell)
powershell -Command "iwr https://fly.io/install.ps1 -useb | iex"

# macOS/Linux
curl -L https://fly.io/install.sh | sh
```

#### 2. Login and Create App
```bash
fly auth login
cd backend
fly launch --name physical-ai-backend --region ord
```

#### 3. Configure Fly.toml
Created automatically, modify as needed:

```toml
app = "physical-ai-backend"
primary_region = "ord"

[env]
  PORT = "8000"
  ENVIRONMENT = "production"

[http_service]
  internal_port = 8000
  force_https = true
  auto_stop_machines = true
  auto_start_machines = true
  min_machines_running = 0

[[vm]]
  cpu_kind = "shared"
  cpus = 1
  memory_mb = 256
```

#### 4. Set Secrets
```bash
fly secrets set \
  DATABASE_URL="postgresql://..." \
  QDRANT_URL="https://..." \
  QDRANT_API_KEY="..." \
  OPENAI_API_KEY="sk-..." \
  ALLOWED_ORIGINS="https://sajid-khan-afridi.github.io,http://localhost:3000"
```

#### 5. Deploy
```bash
fly deploy
```

#### 6. Get URL
```bash
fly status
# URL: https://physical-ai-backend.fly.dev
```

---

## Frontend Deployment (GitHub Pages)

### Step 1: Update API Configuration

Edit `static/js/api-config.js`:

```javascript
const isDevelopment =
  window.location.hostname === 'localhost' ||
  window.location.hostname === '127.0.0.1';

// Replace with your actual backend URL
const apiUrl = isDevelopment
  ? 'http://localhost:8000'
  : 'https://physical-ai-backend.onrender.com';  // Your Render/Railway/Fly.io URL

window.API_CONFIG = {
  baseUrl: apiUrl,
  timeout: 30000,
};
```

### Step 2: Update Docusaurus Config

Verify `docusaurus.config.ts` has correct base URL:

```typescript
export default {
  // ...
  url: 'https://sajid-khan-afridi.github.io',
  baseUrl: '/hackathon1/',
  organizationName: 'sajid-khan-afridi',
  projectName: 'hackathon1',
  deploymentBranch: 'gh-pages',
  trailingSlash: false,
  // ...
};
```

### Step 3: Deploy to GitHub Pages

```bash
# Build the site
npm run build

# Deploy to GitHub Pages
npm run deploy
```

This will:
1. Build static files to `build/` directory
2. Push to `gh-pages` branch
3. GitHub Pages auto-deploys from that branch

### Step 4: Verify GitHub Pages Settings

1. Go to repository → Settings → Pages
2. Source: `Deploy from a branch`
3. Branch: `gh-pages` / `root`
4. Save
5. Wait 2-3 minutes
6. Visit: `https://sajid-khan-afridi.github.io/hackathon1/`

---

## Database & Vector Store Setup

### Neon PostgreSQL (Recommended)

1. Go to [neon.tech](https://neon.tech)
2. Create free account
3. Create new project: `physical-ai`
4. Create database: `physical_ai`
5. Copy connection string:
   ```
   postgresql://user:password@ep-xxx.region.aws.neon.tech/physical_ai?sslmode=require
   ```
6. Add to backend environment variables

### Qdrant Cloud (Vector Database)

1. Go to [cloud.qdrant.io](https://cloud.qdrant.io)
2. Create free account (1GB free tier)
3. Create cluster: `physical-ai-cluster`
4. Copy:
   - Cluster URL: `https://xxx.qdrant.io`
   - API Key: `your-api-key`
5. Add to backend environment variables

---

## Production Checklist

### Backend

- [ ] Remove `--reload` flag from start command
- [ ] Set `DEBUG=false` in environment
- [ ] Set `ENVIRONMENT=production`
- [ ] Configure CORS with production frontend URL
- [ ] Set up PostgreSQL connection (Neon or Render/Railway)
- [ ] Set up Qdrant connection (Qdrant Cloud)
- [ ] Add OpenAI API key
- [ ] Test `/api/v1/health` endpoint
- [ ] Verify database migrations run
- [ ] Enable connection pooling
- [ ] Set up logging (Sentry, LogRocket, etc.)
- [ ] Configure rate limiting (if needed)

### Frontend

- [ ] Update `api-config.js` with production backend URL
- [ ] Verify `baseUrl` in `docusaurus.config.ts`
- [ ] Build successfully (`npm run build`)
- [ ] Deploy to GitHub Pages (`npm run deploy`)
- [ ] Test chat widget on production
- [ ] Verify CORS (no errors in browser console)
- [ ] Test on mobile devices
- [ ] Test dark mode
- [ ] Verify all citations link correctly

### Security

- [ ] All secrets in environment variables (not committed)
- [ ] HTTPS enabled (automatic on Render/Railway/Fly/GitHub Pages)
- [ ] CORS restricted to specific origins
- [ ] Rate limiting configured (optional)
- [ ] SQL injection protection (SQLAlchemy handles this)
- [ ] XSS protection (React handles this)

### Monitoring

- [ ] Health check endpoint working
- [ ] Set up uptime monitoring (UptimeRobot, Pingdom)
- [ ] Configure error tracking (Sentry)
- [ ] Set up analytics (Google Analytics, Plausible)
- [ ] Monitor API usage (OpenAI dashboard)
- [ ] Monitor database usage (Neon dashboard)

---

## Environment Variables Summary

### Backend (.env - DO NOT COMMIT)

```bash
# Database
DATABASE_URL=postgresql://user:password@host:5432/physical_ai

# Qdrant
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-api-key

# OpenAI
OPENAI_API_KEY=sk-...

# CORS
ALLOWED_ORIGINS=https://sajid-khan-afridi.github.io,http://localhost:3000

# App
ENVIRONMENT=production
DEBUG=false
API_V1_STR=/api/v1
PROJECT_NAME=Physical AI RAG API
VERSION=0.1.0
```

### Frontend (api-config.js)

```javascript
// Production backend URL (edit this)
const apiUrl = 'https://physical-ai-backend.onrender.com';
```

---

## Continuous Deployment (CI/CD)

### GitHub Actions for Automatic Deployment

Create `.github/workflows/deploy.yml`:

```yaml
name: Deploy to Production

on:
  push:
    branches: [main]

jobs:
  deploy-frontend:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: '18'
      - run: npm ci
      - run: npm run build
      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build

  deploy-backend:
    runs-on: ubuntu-latest
    steps:
      # Render/Railway/Fly.io auto-deploys on push
      # No additional steps needed
      - run: echo "Backend auto-deploys via platform"
```

---

## Troubleshooting

### Issue: CORS errors on production

**Solution**:
1. Check backend environment variable `ALLOWED_ORIGINS`
2. Must include: `https://sajid-khan-afridi.github.io`
3. Restart backend service after changing environment variables

### Issue: Backend health check fails

**Solution**:
1. Verify database connection string
2. Check Qdrant API key
3. Verify OpenAI API key
4. Check backend logs for detailed errors

### Issue: Frontend can't connect to backend

**Solution**:
1. Verify `api-config.js` has correct backend URL
2. Check browser Network tab for failed requests
3. Verify CORS configuration
4. Test backend directly with `curl`

### Issue: Database connection timeout

**Solution**:
1. Check connection string format
2. Verify firewall rules (allow backend IP)
3. For Neon: ensure `?sslmode=require` in connection string
4. Check database is not paused (free tier auto-pauses)

---

## Cost Estimate (Free Tier)

| Service | Free Tier | Upgrades If Needed |
|---------|-----------|-------------------|
| GitHub Pages | Unlimited static hosting | N/A (always free) |
| Render | 750 hrs/month | $7/month for always-on |
| Railway | $5 credit/month | $5/month usage-based |
| Fly.io | 3 VMs (2340 hrs) | $1.94/VM/month |
| Neon PostgreSQL | 10GB storage, 10 branches | $19/month Pro |
| Qdrant Cloud | 1GB vectors | $25/month Starter |
| OpenAI API | Pay-per-use | Variable ($0.0001/1K tokens) |

**Total Free Tier**: Can run completely free with limitations

**Recommended Paid Plan** (if scaling):
- Render Pro: $7/month
- Qdrant Starter: $25/month
- OpenAI usage: ~$10-50/month (varies)
- **Total**: ~$42-82/month for production-grade

---

## Next Steps

1. **Choose Backend Host**: Render (easiest) / Railway / Fly.io
2. **Deploy Backend**: Follow steps for chosen platform
3. **Set Up Databases**: Neon PostgreSQL + Qdrant Cloud
4. **Update Frontend Config**: Edit `api-config.js` with backend URL
5. **Deploy Frontend**: `npm run deploy`
6. **Test End-to-End**: Chat widget on production
7. **Set Up Monitoring**: UptimeRobot + Sentry
8. **Document**: Update README with production URLs

---

**Ready to deploy!** Start with Render for the simplest setup.
