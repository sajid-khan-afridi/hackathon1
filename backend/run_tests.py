"""Comprehensive test runner for Physical AI RAG system"""

import subprocess
import sys
import os
from pathlib import Path
from datetime import datetime


def print_header(title):
    """Print formatted section header"""
    print("\n" + "="*80)
    print(f"  {title}")
    print("="*80 + "\n")


def run_command(cmd, description):
    """Run command and capture output"""
    print(f"Running: {description}...")
    print(f"Command: {' '.join(cmd)}\n")

    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            cwd=Path(__file__).parent
        )

        print(result.stdout)

        if result.stderr:
            print("STDERR:", result.stderr)

        return result.returncode == 0, result

    except Exception as e:
        print(f"ERROR: {e}")
        return False, None


def main():
    """Run all tests and generate reports"""
    print_header("PHYSICAL AI RAG SYSTEM - COMPREHENSIVE TEST SUITE")
    print(f"Started at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"Working directory: {Path.cwd()}")

    all_results = {
        "unit_tests": None,
        "integration_tests": None,
        "coverage": None
    }

    # =====================================================================
    # 1. UNIT TESTS
    # =====================================================================
    print_header("PHASE 1: UNIT TESTS")

    unit_test_cmd = [
        sys.executable,
        "-m",
        "pytest",
        "tests/test_services/",
        "-v",
        "--tb=short",
        "--color=yes"
    ]

    success, result = run_command(unit_test_cmd, "Unit Tests (Services)")
    all_results["unit_tests"] = success

    if not success:
        print("\n WARNING: Some unit tests failed!")
    else:
        print("\n SUCCESS: All unit tests passed!")

    # =====================================================================
    # 2. INTEGRATION TESTS
    # =====================================================================
    print_header("PHASE 2: INTEGRATION TESTS")

    integration_test_cmd = [
        sys.executable,
        "-m",
        "pytest",
        "tests/test_api/",
        "-v",
        "--tb=short",
        "--color=yes"
    ]

    success, result = run_command(integration_test_cmd, "Integration Tests (API)")
    all_results["integration_tests"] = success

    if not success:
        print("\n WARNING: Some integration tests failed!")
    else:
        print("\n SUCCESS: All integration tests passed!")

    # =====================================================================
    # 3. COVERAGE REPORT
    # =====================================================================
    print_header("PHASE 3: CODE COVERAGE")

    coverage_cmd = [
        sys.executable,
        "-m",
        "pytest",
        "tests/",
        "--cov=app",
        "--cov-report=term-missing",
        "--cov-report=html:htmlcov",
        "--cov-report=json:coverage.json",
        "-v"
    ]

    success, result = run_command(coverage_cmd, "Coverage Report Generation")
    all_results["coverage"] = success

    if success:
        print("\n SUCCESS: Coverage report generated!")
        print("  - HTML report: htmlcov/index.html")
        print("  - JSON report: coverage.json")
    else:
        print("\n WARNING: Coverage report generation had issues!")

    # =====================================================================
    # 4. TEST SUMMARY
    # =====================================================================
    print_header("TEST EXECUTION SUMMARY")

    total_tests = 3
    passed_tests = sum(1 for v in all_results.values() if v)

    print(f"Total Test Phases: {total_tests}")
    print(f"Passed: {passed_tests}")
    print(f"Failed: {total_tests - passed_tests}")
    print(f"\nSuccess Rate: {(passed_tests/total_tests)*100:.1f}%")

    print("\nDetailed Results:")
    for test_name, result in all_results.items():
        status = "PASS" if result else "FAIL"
        symbol = "✓" if result else "✗"
        print(f"  {symbol} {test_name.replace('_', ' ').title()}: {status}")

    # =====================================================================
    # 5. RECOMMENDATIONS
    # =====================================================================
    print_header("RECOMMENDATIONS")

    if all_results["unit_tests"] and all_results["integration_tests"]:
        print("  All tests passed! System is ready for deployment.")
        print("\n  Next Steps:")
        print("  1. Review coverage report (htmlcov/index.html)")
        print("  2. Run performance tests if needed")
        print("  3. Deploy to staging environment")
    else:
        print("  Some tests failed. Please review the output above.")
        print("\n  Next Steps:")
        print("  1. Fix failing tests")
        print("  2. Review error messages")
        print("  3. Re-run this script")

    # =====================================================================
    # 6. EXIT STATUS
    # =====================================================================
    print_header("COMPLETION")
    print(f"Finished at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

    # Exit with appropriate status code
    if all(all_results.values()):
        print("\n ALL TESTS PASSED")
        sys.exit(0)
    else:
        print("\n SOME TESTS FAILED")
        sys.exit(1)


if __name__ == "__main__":
    main()
