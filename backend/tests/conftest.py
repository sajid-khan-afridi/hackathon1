"""Pytest configuration and fixtures"""

import pytest
from fastapi.testclient import TestClient
from app.main import app


@pytest.fixture
def client():
    """
    FastAPI test client fixture

    Returns:
        TestClient for making HTTP requests to the API
    """
    return TestClient(app)


@pytest.fixture
def sample_query():
    """
    Sample query fixture for testing

    Returns:
        Sample user query string
    """
    return "What is ROS2?"


@pytest.fixture
def sample_chat_request():
    """
    Sample ChatRequest fixture

    Returns:
        Dictionary representing a chat request
    """
    return {
        "query": "What is ROS2?"
    }


@pytest.fixture
def sample_chunks():
    """
    Sample chunks fixture for testing retrieval

    Returns:
        List of sample chunk data
    """
    return [
        {
            "content": "ROS2 is the second generation of Robot Operating System...",
            "metadata": {
                "module": "module-01-ros2",
                "chapter": "01-fundamentals",
                "section": "Introduction"
            }
        },
        {
            "content": "ROS2 uses DDS (Data Distribution Service) for communication...",
            "metadata": {
                "module": "module-01-ros2",
                "chapter": "02-architecture",
                "section": "Communication"
            }
        }
    ]
