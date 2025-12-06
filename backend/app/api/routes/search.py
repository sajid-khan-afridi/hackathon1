"""Search endpoint for testing retrieval without generation"""

from fastapi import APIRouter, Depends, Query
from typing import List
from app.models.schemas import SearchRequest, SearchResponse
from app.services.retrieval import RetrievalService
from app.api.deps import get_retrieval_service

router = APIRouter()


@router.post("/search", response_model=SearchResponse)
async def search(
    request: SearchRequest,
    retrieval_service: RetrievalService = Depends(get_retrieval_service),
):
    """
    Search for relevant chunks without generating a response

    Useful for debugging and testing retrieval quality

    Args:
        request: SearchRequest with query and filters
        retrieval_service: Injected retrieval service

    Returns:
        SearchResponse with retrieved chunks and scores
    """
    results = await retrieval_service.hybrid_search(
        query=request.query,
        top_k=request.top_k,
        filters=request.filters
    )
    return SearchResponse(results=results)
