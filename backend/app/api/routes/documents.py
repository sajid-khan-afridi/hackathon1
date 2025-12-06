"""Documents endpoint for listing and filtering book content"""

from fastapi import APIRouter, Depends, Query
from typing import List, Optional
from app.models.schemas import DocumentResponse
from app.services.storage import StorageService
from app.api.deps import get_storage_service

router = APIRouter()


@router.get("/documents", response_model=List[DocumentResponse])
async def list_documents(
    module: Optional[str] = Query(None, description="Filter by module name"),
    chapter: Optional[str] = Query(None, description="Filter by chapter name"),
    storage_service: StorageService = Depends(get_storage_service),
):
    """
    List documents with optional filtering

    Args:
        module: Optional module filter
        chapter: Optional chapter filter
        storage_service: Injected storage service

    Returns:
        List of DocumentResponse objects
    """
    documents = await storage_service.get_documents(module=module, chapter=chapter)
    return documents
