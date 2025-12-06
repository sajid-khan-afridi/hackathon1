"""Tests for health check API endpoint"""

import pytest
from fastapi.testclient import TestClient


def test_health_endpoint_exists(client: TestClient):
    """Test that health endpoint is accessible"""
    response = client.get("/api/v1/health")
    assert response.status_code == 200


def test_health_response_structure(client: TestClient):
    """Test health response has required fields"""
    response = client.get("/api/v1/health")
    data = response.json()

    assert "status" in data
    assert "services" in data
    assert isinstance(data["services"], dict)


def test_health_includes_database_status(client: TestClient):
    """Test health check includes database status"""
    response = client.get("/api/v1/health")
    data = response.json()

    assert "database" in data["services"]


def test_health_includes_qdrant_status(client: TestClient):
    """Test health check includes Qdrant status"""
    response = client.get("/api/v1/health")
    data = response.json()

    assert "qdrant" in data["services"]
