#!/usr/bin/env python3
"""
Initialize Physical AI RAG project structure
Creates all necessary directories and placeholder files
"""

import os
import sys
from pathlib import Path
from typing import List, Dict

def create_directory_structure(base_path: Path, structure: Dict):
    """
    Recursively create directory structure

    Args:
        base_path: Root directory
        structure: Dict mapping directory names to subdirectories or files
    """
    for name, content in structure.items():
        path = base_path / name

        if isinstance(content, dict):
            # It's a directory
            path.mkdir(parents=True, exist_ok=True)
            print(f"✓ Created directory: {path}")

            # Recursively create subdirectories
            if content:
                create_directory_structure(path, content)
        else:
            # It's a file (content is file content or None)
            path.parent.mkdir(parents=True, exist_ok=True)
            if content is not None:
                path.write_text(content)
                print(f"✓ Created file: {path}")

def get_project_structure() -> Dict:
    """Define the complete project structure"""

    return {
        "frontend": {
            "docs": {
                "module-01-ros2": {},
                "module-02-gazebo-unity": {},
                "module-03-nvidia-isaac": {},
                "module-04-vla-conversational": {},
                "intro.md": "# Physical AI Book\n\nWelcome to the Physical AI book!\n",
            },
            "src": {
                "components": {
                    "ChatInterface": {
                        "ChatInterface.tsx": None,
                        "ChatMessage.tsx": None,
                        "ChatInterface.module.css": None,
                    },
                },
                "pages": {},
                "services": {
                    "api.ts": None,
                },
                "theme": {},
            },
            "static": {
                "img": {},
                "js": {},
            },
            "package.json": None,
            "docusaurus.config.ts": None,
            "sidebars.ts": None,
            "tsconfig.json": None,
        },
        "backend": {
            "app": {
                "__init__.py": "",
                "main.py": '"""FastAPI application entry point"""\n',
                "api": {
                    "__init__.py": "",
                    "routes": {
                        "__init__.py": "",
                        "chat.py": None,
                        "documents.py": None,
                        "health.py": None,
                    },
                    "deps.py": None,
                },
                "core": {
                    "__init__.py": "",
                    "config.py": None,
                    "security.py": None,
                },
                "models": {
                    "__init__.py": "",
                    "database.py": None,
                    "schemas.py": None,
                },
                "services": {
                    "__init__.py": "",
                    "rag.py": None,
                    "embeddings.py": None,
                    "retrieval.py": None,
                    "storage.py": None,
                },
                "utils": {
                    "__init__.py": "",
                    "chunking.py": None,
                    "logging.py": None,
                },
            },
            "migrations": {
                "versions": {},
            },
            "tests": {
                "__init__.py": "",
                "conftest.py": None,
                "test_api": {},
                "test_services": {},
            },
            "scripts": {
                "ingest_documents.py": None,
                "init_db.py": None,
            },
            "requirements.txt": None,
            "requirements-dev.txt": None,
            "pyproject.toml": None,
        },
        "shared": {
            "docker": {
                "docker-compose.yml": None,
                "Dockerfile.backend": None,
                "Dockerfile.frontend": None,
            },
            "config": {
                ".env.example": None,
            },
        },
        ".github": {
            "workflows": {
                "backend-tests.yml": None,
                "frontend-build.yml": None,
            },
        },
        "docs-raw": {},
        ".gitignore": None,
        "README.md": "# Physical AI Book - RAG Chatbot\n\nA comprehensive book on Physical AI with an integrated RAG chatbot.\n",
        "Makefile": None,
    }

def create_env_example():
    """Create .env.example file with all required variables"""
    return """# Backend API
API_URL=http://localhost:8000

# Database
DATABASE_URL=postgresql://user:password@localhost:5432/physical_ai
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=your-qdrant-api-key

# OpenAI
OPENAI_API_KEY=your-openai-api-key

# Frontend
FRONTEND_URL=http://localhost:3000

# CORS (comma-separated)
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:8000
"""

def create_gitignore():
    """Create .gitignore file"""
    return """# Environment
.env
.env.local

# Python
__pycache__/
*.py[cod]
*$py.class
*.so
.Python
env/
venv/
ENV/
build/
develop-eggs/
dist/
downloads/
eggs/
.eggs/
lib/
lib64/
parts/
sdist/
var/
wheels/
*.egg-info/
.installed.cfg
*.egg

# Node
node_modules/
npm-debug.log*
yarn-debug.log*
yarn-error.log*
.pnpm-debug.log*

# Docusaurus
.docusaurus/
.cache-loader/
build/

# IDEs
.vscode/
.idea/
*.swp
*.swo
*~

# OS
.DS_Store
Thumbs.db

# Logs
*.log

# Database
*.db
*.sqlite
"""

def create_makefile():
    """Create Makefile for common commands"""
    return """# Physical AI Book - Development Commands

.PHONY: install dev build test clean help

help:
\t@echo "Available commands:"
\t@echo "  make install    - Install all dependencies"
\t@echo "  make dev        - Run development servers"
\t@echo "  make build      - Build for production"
\t@echo "  make test       - Run tests"
\t@echo "  make clean      - Clean build artifacts"

install:
\t@echo "Installing frontend dependencies..."
\tcd frontend && npm install
\t@echo "Installing backend dependencies..."
\tcd backend && pip install -r requirements.txt
\t@echo "Done!"

dev:
\t@echo "Starting development servers..."
\t@make -j2 dev-frontend dev-backend

dev-frontend:
\tcd frontend && npm start

dev-backend:
\tcd backend && uvicorn app.main:app --reload --port 8000

build:
\t@echo "Building frontend..."
\tcd frontend && npm run build
\t@echo "Build complete!"

test:
\t@echo "Running frontend tests..."
\tcd frontend && npm test
\t@echo "Running backend tests..."
\tcd backend && pytest
\t@echo "All tests complete!"

clean:
\t@echo "Cleaning build artifacts..."
\tcd frontend && rm -rf node_modules build .docusaurus
\tcd backend && rm -rf __pycache__ .pytest_cache
\t@echo "Clean complete!"

validate:
\t@echo "Validating setup..."
\tpython scripts/validate_setup.py
"""

def main():
    """Main entry point"""
    print("=" * 60)
    print("Physical AI RAG Project Structure Initialization")
    print("=" * 60)
    print()

    # Get current directory
    base_path = Path.cwd()

    # Confirm with user
    print(f"This will create the project structure in: {base_path}")
    response = input("Continue? (y/n): ")

    if response.lower() != 'y':
        print("Aborted.")
        return 1

    print()

    # Create structure
    structure = get_project_structure()
    create_directory_structure(base_path, structure)

    # Create special files with content
    print("\nCreating configuration files...")

    env_example = base_path / "shared" / "config" / ".env.example"
    env_example.write_text(create_env_example())
    print(f"✓ Created: {env_example}")

    gitignore = base_path / ".gitignore"
    if not gitignore.exists():
        gitignore.write_text(create_gitignore())
        print(f"✓ Created: {gitignore}")

    makefile = base_path / "Makefile"
    makefile.write_text(create_makefile())
    print(f"✓ Created: {makefile}")

    print()
    print("=" * 60)
    print("✓ Project structure created successfully!")
    print("=" * 60)
    print()
    print("Next steps:")
    print("  1. Copy .env.example to .env and fill in your values")
    print("  2. Run 'make install' to install dependencies")
    print("  3. Start Docker services: docker-compose -f shared/docker/docker-compose.yml up -d")
    print("  4. Initialize database: cd backend && python scripts/init_db.py")
    print("  5. Run 'make dev' to start development servers")
    print()

    return 0

if __name__ == "__main__":
    sys.exit(main())
