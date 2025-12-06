#!/usr/bin/env python3
"""
Validate Physical AI RAG setup
Checks all components and dependencies
"""

import os
import sys
from pathlib import Path
import json
import subprocess

class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    END = '\033[0m'

def print_status(message, status="info"):
    """Print colored status message"""
    colors = {
        "success": Colors.GREEN + "✓" + Colors.END,
        "error": Colors.RED + "✗" + Colors.END,
        "warning": Colors.YELLOW + "⚠" + Colors.END,
        "info": Colors.BLUE + "ℹ" + Colors.END,
    }
    print(f"{colors.get(status, '')} {message}")

def check_directory_structure():
    """Validate project directory structure"""
    print_status("Checking directory structure...", "info")

    required_dirs = [
        "frontend",
        "backend",
        "backend/app",
        "backend/app/api",
        "backend/app/services",
        "backend/app/models",
    ]

    missing = []
    for dir_path in required_dirs:
        if not Path(dir_path).exists():
            missing.append(dir_path)
            print_status(f"Missing directory: {dir_path}", "error")
        else:
            print_status(f"Found: {dir_path}", "success")

    return len(missing) == 0

def check_frontend_setup():
    """Check Docusaurus frontend"""
    print_status("\nChecking frontend setup...", "info")

    checks = []

    # Check package.json
    package_json = Path("frontend/package.json")
    if package_json.exists():
        print_status("package.json found", "success")
        with open(package_json) as f:
            pkg = json.load(f)
            if "@docusaurus/core" in pkg.get("dependencies", {}):
                print_status("Docusaurus dependency found", "success")
                checks.append(True)
            else:
                print_status("Docusaurus not in dependencies", "error")
                checks.append(False)
    else:
        print_status("package.json not found", "error")
        checks.append(False)

    # Check docusaurus.config
    config_files = ["frontend/docusaurus.config.ts", "frontend/docusaurus.config.js"]
    config_found = any(Path(f).exists() for f in config_files)
    if config_found:
        print_status("Docusaurus config found", "success")
        checks.append(True)
    else:
        print_status("Docusaurus config not found", "error")
        checks.append(False)

    # Check ChatInterface component
    chat_interface = Path("frontend/src/components/ChatInterface")
    if chat_interface.exists():
        print_status("ChatInterface component found", "success")
        checks.append(True)
    else:
        print_status("ChatInterface component not found", "warning")
        checks.append(False)

    return all(checks)

def check_backend_setup():
    """Check FastAPI backend"""
    print_status("\nChecking backend setup...", "info")

    checks = []

    # Check requirements.txt
    requirements = Path("backend/requirements.txt")
    if requirements.exists():
        print_status("requirements.txt found", "success")
        with open(requirements) as f:
            content = f.read()
            required_packages = ["fastapi", "uvicorn", "openai", "qdrant-client", "sqlalchemy"]
            for pkg in required_packages:
                if pkg in content.lower():
                    print_status(f"  ✓ {pkg}", "success")
                    checks.append(True)
                else:
                    print_status(f"  ✗ {pkg} not found", "error")
                    checks.append(False)
    else:
        print_status("requirements.txt not found", "error")
        checks.append(False)

    # Check main.py
    main_py = Path("backend/app/main.py")
    if main_py.exists():
        print_status("main.py found", "success")
        checks.append(True)
    else:
        print_status("main.py not found", "error")
        checks.append(False)

    # Check RAG service
    rag_service = Path("backend/app/services/rag.py")
    if rag_service.exists():
        print_status("RAG service found", "success")
        checks.append(True)
    else:
        print_status("RAG service not found", "warning")
        checks.append(False)

    return all(checks)

def check_environment_variables():
    """Check required environment variables"""
    print_status("\nChecking environment variables...", "info")

    required_vars = [
        "DATABASE_URL",
        "QDRANT_URL",
        "OPENAI_API_KEY",
    ]

    env_file = Path(".env")
    if not env_file.exists():
        print_status(".env file not found", "error")
        return False

    checks = []
    with open(env_file) as f:
        env_content = f.read()
        for var in required_vars:
            if var in env_content and not f"{var}=" in env_content.replace(f"{var}=", ""):
                print_status(f"{var} configured", "success")
                checks.append(True)
            else:
                print_status(f"{var} not configured", "error")
                checks.append(False)

    return all(checks)

def check_database_connection():
    """Check database connectivity"""
    print_status("\nChecking database connections...", "info")

    # This would require actual database clients, so we'll just check configuration
    print_status("Database connection check requires running backend", "info")
    return True

def check_docker_compose():
    """Check Docker Compose setup"""
    print_status("\nChecking Docker Compose...", "info")

    docker_compose = Path("shared/docker/docker-compose.yml")
    if docker_compose.exists():
        print_status("docker-compose.yml found", "success")
        return True
    else:
        print_status("docker-compose.yml not found", "warning")
        return False

def main():
    """Run all validation checks"""
    print(f"\n{Colors.BLUE}{'='*60}{Colors.END}")
    print(f"{Colors.BLUE}Physical AI RAG Setup Validation{Colors.END}")
    print(f"{Colors.BLUE}{'='*60}{Colors.END}\n")

    results = {
        "Directory Structure": check_directory_structure(),
        "Frontend Setup": check_frontend_setup(),
        "Backend Setup": check_backend_setup(),
        "Environment Variables": check_environment_variables(),
        "Docker Compose": check_docker_compose(),
    }

    # Print summary
    print(f"\n{Colors.BLUE}{'='*60}{Colors.END}")
    print(f"{Colors.BLUE}Validation Summary{Colors.END}")
    print(f"{Colors.BLUE}{'='*60}{Colors.END}\n")

    all_passed = True
    for check, passed in results.items():
        status = "success" if passed else "error"
        print_status(f"{check}: {'PASSED' if passed else 'FAILED'}", status)
        if not passed:
            all_passed = False

    print()

    if all_passed:
        print_status("All checks passed! ✓", "success")
        return 0
    else:
        print_status("Some checks failed. Please review the output above.", "error")
        return 1

if __name__ == "__main__":
    sys.exit(main())
