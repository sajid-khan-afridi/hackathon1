"""FastAPI application entry point"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.core.config import settings
from app.api.routes import chat, documents, health, search

# Create FastAPI app
app = FastAPI(
    title=settings.PROJECT_NAME,
    version=settings.VERSION,
    openapi_url=f"{settings.API_V1_STR}/openapi.json",
    docs_url=f"{settings.API_V1_STR}/docs",
    redoc_url=f"{settings.API_V1_STR}/redoc",
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE"],
    allow_headers=["*"],
)

# Include routers
app.include_router(
    chat.router,
    prefix=settings.API_V1_STR,
    tags=["chat"]
)
app.include_router(
    documents.router,
    prefix=settings.API_V1_STR,
    tags=["documents"]
)
app.include_router(
    health.router,
    prefix=settings.API_V1_STR,
    tags=["health"]
)
app.include_router(
    search.router,
    prefix=settings.API_V1_STR,
    tags=["search"]
)


@app.get("/")
async def root():
    """Root endpoint"""
    return {
        "message": "Physical AI RAG API",
        "version": settings.VERSION,
        "docs": f"{settings.API_V1_STR}/docs"
    }


@app.on_event("startup")
async def startup_event():
    """Application startup event"""
    # TODO: Initialize database connection pool
    # TODO: Initialize Qdrant client
    # TODO: Setup logging
    pass


@app.on_event("shutdown")
async def shutdown_event():
    """Application shutdown event"""
    # TODO: Close database connections
    # TODO: Close Qdrant client
    pass
