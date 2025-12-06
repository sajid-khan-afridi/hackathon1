"""Chat endpoint for RAG-based question answering"""

import logging
from fastapi import APIRouter, Depends, HTTPException, status
from app.models.schemas import ChatRequest, ChatResponse
from app.services.rag import RAGService
from app.api.deps import get_rag_service

# Configure logging
logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("/chat", response_model=ChatResponse)
async def chat(
    request: ChatRequest,
    rag_service: RAGService = Depends(get_rag_service),
):
    """
    Handle chat requests with RAG pipeline

    Args:
        request: ChatRequest with user query, conversation_id, and top_k
        rag_service: Injected RAG service

    Returns:
        ChatResponse with generated answer, citations, and performance metrics

    Raises:
        HTTPException: If query processing fails
            - 400: Invalid request (validation errors)
            - 500: Internal server error (RAG pipeline failure)
            - 503: Service unavailable (OpenAI, Qdrant, or database down)
    """
    try:
        logger.info(f"Processing chat request: query='{request.query[:50]}...', conversation_id={request.conversation_id}")

        # Generate response using RAG pipeline
        response = await rag_service.generate_response(
            query=request.query,
            conversation_id=request.conversation_id,
            top_k=request.top_k
        )

        logger.info(f"Successfully generated response: sources={len(response.sources)}, time={response.total_time_ms}ms")
        return response

    except ValueError as e:
        # Validation errors (malformed query, invalid parameters)
        logger.error(f"Validation error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Invalid request: {str(e)}"
        )

    except ConnectionError as e:
        # Service connectivity issues (Qdrant, OpenAI, database)
        logger.error(f"Service unavailable: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="One or more required services are currently unavailable. Please try again later."
        )

    except Exception as e:
        # Unexpected errors
        logger.exception(f"Unexpected error in chat endpoint: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An unexpected error occurred while processing your request. Please try again."
        )
