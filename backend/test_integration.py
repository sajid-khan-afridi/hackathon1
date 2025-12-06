"""
Integration test script for FastAPI + Docusaurus RAG chatbot

Tests:
1. Health endpoint - checks all services
2. Chat endpoint - sends a test query
3. Documents endpoint - lists available documents
4. Search endpoint - tests raw retrieval

Usage:
    python backend/test_integration.py
"""

import requests
import json
import sys
from typing import Dict, Any

# Configuration
BASE_URL = "http://localhost:8000"
API_V1 = f"{BASE_URL}/api/v1"

# Colors for terminal output
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
RESET = "\033[0m"


def print_success(message: str):
    """Print success message in green"""
    print(f"{GREEN}✓ {message}{RESET}")


def print_error(message: str):
    """Print error message in red"""
    print(f"{RED}✗ {message}{RESET}")


def print_info(message: str):
    """Print info message in blue"""
    print(f"{BLUE}ℹ {message}{RESET}")


def print_warning(message: str):
    """Print warning message in yellow"""
    print(f"{YELLOW}⚠ {message}{RESET}")


def test_health() -> bool:
    """Test health endpoint"""
    print_info("Testing health endpoint...")

    try:
        response = requests.get(f"{API_V1}/health", timeout=10)

        if response.status_code == 200:
            data = response.json()
            print_success(f"Health check passed: {response.status_code}")
            print(f"  Status: {data.get('status')}")
            print(f"  Version: {data.get('version')}")
            print(f"  Services:")
            for service, status in data.get('services', {}).items():
                icon = "✓" if status == "healthy" else "✗"
                color = GREEN if status == "healthy" else RED
                print(f"    {color}{icon} {service}: {status}{RESET}")

            return data.get('status') in ['healthy', 'degraded']
        else:
            print_error(f"Health check failed: {response.status_code}")
            print(f"  Response: {response.text}")
            return False

    except requests.exceptions.RequestException as e:
        print_error(f"Health check failed: {str(e)}")
        return False


def test_chat() -> bool:
    """Test chat endpoint"""
    print_info("Testing chat endpoint...")

    test_query = "What is ROS2?"

    try:
        response = requests.post(
            f"{API_V1}/chat",
            json={"query": test_query, "top_k": 5},
            timeout=30,
        )

        if response.status_code == 200:
            data = response.json()
            print_success(f"Chat request successful: {response.status_code}")
            print(f"  Query: {test_query}")
            print(f"  Response: {data.get('response')[:100]}...")
            print(f"  Sources: {len(data.get('sources', []))}")
            print(f"  Query Type: {data.get('query_type')}")
            print(f"  Total Time: {data.get('total_time_ms')}ms")

            if data.get('sources'):
                print(f"  First source:")
                source = data['sources'][0]
                print(f"    Module: {source.get('module')}")
                print(f"    Chapter: {source.get('chapter')}")
                print(f"    Section: {source.get('section')}")

            return True
        else:
            print_error(f"Chat request failed: {response.status_code}")
            print(f"  Response: {response.text}")
            return False

    except requests.exceptions.RequestException as e:
        print_error(f"Chat request failed: {str(e)}")
        return False


def test_documents() -> bool:
    """Test documents endpoint"""
    print_info("Testing documents endpoint...")

    try:
        response = requests.get(f"{API_V1}/documents", timeout=10)

        if response.status_code == 200:
            data = response.json()
            print_success(f"Documents request successful: {response.status_code}")
            print(f"  Total documents: {len(data)}")

            if data:
                print(f"  First document:")
                doc = data[0]
                print(f"    Title: {doc.get('title')}")
                print(f"    Module: {doc.get('module')}")
                print(f"    Chapter: {doc.get('chapter')}")

            return True
        else:
            print_error(f"Documents request failed: {response.status_code}")
            print(f"  Response: {response.text}")
            return False

    except requests.exceptions.RequestException as e:
        print_error(f"Documents request failed: {str(e)}")
        return False


def test_search() -> bool:
    """Test search endpoint"""
    print_info("Testing search endpoint...")

    test_query = "ROS2 publisher"

    try:
        response = requests.post(
            f"{API_V1}/search",
            json={"query": test_query, "top_k": 3},
            timeout=30,
        )

        if response.status_code == 200:
            data = response.json()
            print_success(f"Search request successful: {response.status_code}")
            print(f"  Query: {test_query}")
            print(f"  Results: {len(data.get('results', []))}")

            if data.get('results'):
                print(f"  First result:")
                result = data['results'][0]
                print(f"    Score: {result.get('score'):.3f}")
                print(f"    Content preview: {result.get('content')[:100]}...")

            return True
        else:
            print_error(f"Search request failed: {response.status_code}")
            print(f"  Response: {response.text}")
            return False

    except requests.exceptions.RequestException as e:
        print_error(f"Search request failed: {str(e)}")
        return False


def main():
    """Run all integration tests"""
    print("\n" + "=" * 70)
    print("Physical AI RAG Chatbot - Integration Tests")
    print("=" * 70 + "\n")

    results = {
        "Health Check": test_health(),
        "Chat Endpoint": test_chat(),
        "Documents Endpoint": test_documents(),
        "Search Endpoint": test_search(),
    }

    print("\n" + "=" * 70)
    print("Test Summary")
    print("=" * 70 + "\n")

    passed = sum(1 for result in results.values() if result)
    total = len(results)

    for test_name, result in results.items():
        status = "PASSED" if result else "FAILED"
        color = GREEN if result else RED
        print(f"{color}{status:8}{RESET} {test_name}")

    print(f"\n{passed}/{total} tests passed\n")

    if passed == total:
        print_success("All tests passed!")
        sys.exit(0)
    else:
        print_error(f"{total - passed} test(s) failed")
        sys.exit(1)


if __name__ == "__main__":
    main()
