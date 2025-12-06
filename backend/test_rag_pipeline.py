"""Test runner for RAG pipeline validation"""

import asyncio
import logging
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from app.services.rag import test_rag_pipeline, test_embeddings, test_retrieval

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)


async def main():
    """Run all RAG pipeline tests"""
    print("\n" + "="*80)
    print("PHYSICAL AI RAG PIPELINE - COMPREHENSIVE TEST SUITE")
    print("="*80)

    tests_passed = 0
    tests_failed = 0

    # Test 1: Embeddings Service
    print("\n[1/3] Testing Embeddings Service...")
    try:
        await test_embeddings()
        tests_passed += 1
    except Exception as e:
        print(f"✗ Embeddings test failed: {e}")
        tests_failed += 1

    # Test 2: Retrieval Service
    print("\n[2/3] Testing Retrieval Service...")
    try:
        await test_retrieval()
        tests_passed += 1
    except Exception as e:
        print(f"✗ Retrieval test failed: {e}")
        tests_failed += 1

    # Test 3: End-to-End RAG Pipeline
    print("\n[3/3] Testing End-to-End RAG Pipeline...")
    try:
        await test_rag_pipeline()
        tests_passed += 1
    except Exception as e:
        print(f"✗ RAG pipeline test failed: {e}")
        tests_failed += 1

    # Summary
    print("\n" + "="*80)
    print("TEST SUMMARY")
    print("="*80)
    print(f"✓ Passed: {tests_passed}/3")
    print(f"✗ Failed: {tests_failed}/3")

    if tests_failed == 0:
        print("\n🎉 ALL TESTS PASSED!")
        return 0
    else:
        print(f"\n⚠️  {tests_failed} test(s) failed")
        return 1


if __name__ == "__main__":
    exit_code = asyncio.run(main())
    sys.exit(exit_code)
