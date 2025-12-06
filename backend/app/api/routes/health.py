"""Health check endpoint for monitoring system status"""

import logging
from datetime import datetime
from fastapi import APIRouter, Depends
from app.models.schemas import HealthResponse
from app.api.deps import get_storage_service, get_embeddings_service
from app.services.storage import StorageService
from app.services.embeddings import EmbeddingsService
from app.core.config import settings

# Configure logging
logger = logging.getLogger(__name__)

router = APIRouter()


@router.get("/health", response_model=HealthResponse)
async def health_check(
    storage_service: StorageService = Depends(get_storage_service),
    embeddings_service: EmbeddingsService = Depends(get_embeddings_service),
):
    """
    Health check endpoint

    Checks connectivity and health of all critical services:
    - PostgreSQL database (Neon)
    - Qdrant vector database
    - OpenAI API

    Returns:
        HealthResponse with overall status, individual service statuses, version, and timestamp

    Overall Status:
        - "healthy": All services operational
        - "degraded": Some services have issues but system can still function
        - "unhealthy": Critical services are down
    """
    services = {}
    critical_services_healthy = []

    try:
        # Check database connection (critical)
        try:
            db_healthy = await storage_service.check_database_health()
            services["database"] = "healthy" if db_healthy else "unhealthy"
            critical_services_healthy.append(db_healthy)
        except Exception as e:
            logger.error(f"Database health check failed: {str(e)}")
            services["database"] = "unhealthy"
            critical_services_healthy.append(False)

        # Check Qdrant connection (critical)
        try:
            qdrant_healthy = await storage_service.check_qdrant_health()
            services["qdrant"] = "healthy" if qdrant_healthy else "unhealthy"
            critical_services_healthy.append(qdrant_healthy)
        except Exception as e:
            logger.error(f"Qdrant health check failed: {str(e)}")
            services["qdrant"] = "unhealthy"
            critical_services_healthy.append(False)

        # Check OpenAI API connection (critical)
        try:
            openai_healthy = await embeddings_service.check_openai_health()
            services["openai"] = "healthy" if openai_healthy else "unhealthy"
            critical_services_healthy.append(openai_healthy)
        except Exception as e:
            logger.error(f"OpenAI health check failed: {str(e)}")
            services["openai"] = "unhealthy"
            critical_services_healthy.append(False)

        # Determine overall status
        if all(critical_services_healthy):
            overall_status = "healthy"
        elif any(critical_services_healthy):
            overall_status = "degraded"
        else:
            overall_status = "unhealthy"

        return HealthResponse(
            status=overall_status,
            services=services,
            version=settings.VERSION,
            timestamp=datetime.utcnow()
        )

    except Exception as e:
        logger.exception(f"Unexpected error in health check: {str(e)}")
        return HealthResponse(
            status="unhealthy",
            services={
                "database": "error",
                "qdrant": "error",
                "openai": "error",
                "error": str(e)
            },
            version=settings.VERSION,
            timestamp=datetime.utcnow()
        )
