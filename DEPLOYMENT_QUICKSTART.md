# 🚀 Quick Deployment Guide

Get your Physical AI RAG Chatbot live in production in **under 30 minutes**!

---

## 📋 Prerequisites

- [ ] GitHub account
- [ ] OpenAI API key ([get one here](https://platform.openai.com/api-keys))
- [ ] Neon PostgreSQL account ([sign up](https://neon.tech))
- [ ] Qdrant Cloud account ([sign up](https://cloud.qdrant.io))
- [ ] Render account ([sign up](https://render.com))

---

## 🎯 Step-by-Step Deployment

### Step 1: Set Up Databases (15 minutes)

#### A. Neon PostgreSQL
1. Go to [neon.tech](https://neon.tech) and sign in
2. Create new project: **physical-ai**
3. Create database: **physical_ai**
4. Copy connection string (starts with `postgresql://...`)
5. Save it for later

#### B. Qdrant Cloud
1. Go to [cloud.qdrant.io](https://cloud.qdrant.io) and sign in
2. Create cluster: **physical-ai-cluster**
3. Region: Choose closest to your users
4. Plan: **Free** (1GB)
5. Copy:
   - **Cluster URL**: `https://xxx-xxx.qdrant.io`
   - **API Key**: from cluster settings
6. Save both for later

---

### Step 2: Deploy Backend to Render (10 minutes)

1. **Sign in to Render**: [render.com](https://render.com) with GitHub

2. **Create Web Service**:
   - Click **New** → **Web Service**
   - Connect repository: `sajid-khan-afridi/hackathon1`
   - Configure:
     - **Name**: `physical-ai-backend`
     - **Region**: Oregon (or closest to you)
     - **Branch**: `main`
     - **Root Directory**: `backend`
     - **Runtime**: `Python 3`
     - **Build Command**: `pip install -r requirements.txt`
     - **Start Command**: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`
     - **Plan**: `Free`

3. **Set Environment Variables**:
   Click **Advanced** → **Add Environment Variable** for each:

   ```bash
   DATABASE_URL=postgresql://your-neon-connection-string
   QDRANT_URL=https://your-cluster.qdrant.io
   QDRANT_API_KEY=your-qdrant-api-key
   OPENAI_API_KEY=sk-your-openai-key
   ALLOWED_ORIGINS=https://sajid-khan-afridi.github.io,http://localhost:3000
   ENVIRONMENT=production
   DEBUG=false
   ```

4. **Deploy**:
   - Click **Create Web Service**
   - Wait 2-5 minutes for build
   - Copy service URL when "Live" (e.g., `https://physical-ai-backend.onrender.com`)

5. **Test Backend**:
   ```bash
   curl https://physical-ai-backend.onrender.com/api/v1/health
   ```
   Should return `{"status":"healthy"}`

---

### Step 3: Update Frontend Config (2 minutes)

1. **Edit `static/js/api-config.js`**:
   ```javascript
   const apiUrl = isDevelopment
     ? 'http://localhost:8000'
     : 'https://physical-ai-backend.onrender.com';  // YOUR RENDER URL HERE
   ```

2. **Commit changes**:
   ```bash
   git add static/js/api-config.js
   git commit -m "Update production API URL"
   git push origin main
   ```

---

### Step 4: Deploy Frontend to GitHub Pages (3 minutes)

1. **Build and Deploy**:
   ```bash
   npm run build
   npm run deploy
   ```

2. **Verify GitHub Pages**:
   - Go to: `https://github.com/sajid-khan-afridi/hackathon1/settings/pages`
   - Source: `gh-pages` branch
   - Wait 2-3 minutes
   - Visit: `https://sajid-khan-afridi.github.io/hackathon1/`

3. **Test Chat Widget**:
   - Look for floating purple button (bottom-right)
   - Click to open chat
   - Send test message: "What is ROS2?"
   - Should get AI response with citations

---

## ✅ Verification Checklist

- [ ] Backend health check returns `healthy`
- [ ] Frontend loads at GitHub Pages URL
- [ ] Chat widget appears on all pages
- [ ] Can send messages and get responses
- [ ] Citations link to correct documentation sections
- [ ] No CORS errors in browser console
- [ ] Works on mobile devices
- [ ] Dark mode works correctly

---

## 🔧 Troubleshooting

### Backend Issues

**Problem**: Health check returns `unhealthy`
```bash
# Check backend logs
# In Render dashboard → Logs
# Look for database connection errors
```

**Solution**: Verify environment variables are set correctly

---

**Problem**: CORS errors in browser
```
Access to fetch at 'https://physical-ai-backend.onrender.com' from origin 'https://sajid-khan-afridi.github.io' has been blocked by CORS policy
```

**Solution**: Add GitHub Pages URL to `ALLOWED_ORIGINS`:
```bash
ALLOWED_ORIGINS=https://sajid-khan-afridi.github.io,http://localhost:3000
```

---

### Frontend Issues

**Problem**: Chat widget not appearing

**Solution**: Clear browser cache and hard refresh (Ctrl+Shift+R)

---

**Problem**: "Service unavailable" error

**Solution**:
1. Check if backend is running (visit backend URL)
2. Verify API URL in `api-config.js` is correct
3. Check backend logs for errors

---

## 📊 Monitoring

### Set Up Uptime Monitoring (Optional)

1. **UptimeRobot** (Free):
   - [uptimerobot.com](https://uptimerobot.com)
   - Add monitor: `https://physical-ai-backend.onrender.com/api/v1/health`
   - Get alerts via email if backend goes down

2. **Error Tracking with Sentry** (Optional):
   - [sentry.io](https://sentry.io)
   - Add Sentry SDK to backend
   - Track errors in production

---

## 💰 Cost Breakdown

### Free Tier (Recommended for Starting)

| Service | Free Tier | Limitations |
|---------|-----------|-------------|
| Render | 750 hrs/month | Sleeps after 15min inactivity |
| GitHub Pages | Unlimited | 100GB bandwidth/month |
| Neon PostgreSQL | 10GB storage | 1 project, unlimited branches |
| Qdrant Cloud | 1GB vectors | ~100K embeddings |
| OpenAI API | Pay-per-use | ~$0.0001/1K tokens |

**Monthly Cost**: $0 - $10 (mostly OpenAI usage)

### Paid Tier (For Production Use)

| Service | Plan | Cost |
|---------|------|------|
| Render Pro | Always-on | $7/month |
| Neon Pro | More storage | $19/month |
| Qdrant Starter | 4GB vectors | $25/month |
| OpenAI API | Usage-based | $20-100/month |

**Monthly Cost**: $71 - $151/month

---

## 🎉 You're Live!

Your Physical AI RAG Chatbot is now deployed and accessible at:

- **Frontend**: https://sajid-khan-afridi.github.io/hackathon1/
- **Backend**: https://physical-ai-backend.onrender.com
- **API Docs**: https://physical-ai-backend.onrender.com/api/v1/docs

---

## 🔄 Continuous Deployment

Every time you push to `main` branch:
- ✅ GitHub Actions automatically deploys frontend to GitHub Pages
- ✅ Render automatically rebuilds and deploys backend

No manual deployment needed!

---

## 📚 Next Steps

1. **Populate Database**: Ingest Physical AI book content
2. **Custom Domain** (Optional): Set up custom domain for GitHub Pages
3. **Analytics**: Add Google Analytics or Plausible
4. **A/B Testing**: Test different RAG parameters
5. **User Feedback**: Add feedback mechanism to chat

---

## 🆘 Need Help?

- **Documentation**: See [PRODUCTION_DEPLOYMENT_GUIDE.md](./PRODUCTION_DEPLOYMENT_GUIDE.md)
- **Issues**: [github.com/sajid-khan-afridi/hackathon1/issues](https://github.com/sajid-khan-afridi/hackathon1/issues)
- **Render Docs**: [render.com/docs](https://render.com/docs)
- **Neon Docs**: [neon.tech/docs](https://neon.tech/docs)
- **Qdrant Docs**: [qdrant.tech/documentation](https://qdrant.tech/documentation)

---

**Happy Deploying!** 🚀
