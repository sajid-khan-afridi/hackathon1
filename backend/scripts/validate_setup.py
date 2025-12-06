"""System validation script for health checks"""

import asyncio
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.core.config import settings
from app.services.storage import StorageService


async def validate_system():
    """
    Validate system configuration and connectivity

    Checks:
        1. Environment variables loaded
        2. Database connection
        3. Qdrant connection
        4. OpenAI API key configured
    """
    print("=" * 60)
    print("Physical AI RAG System Validation")
    print("=" * 60)

    all_checks_passed = True

    # Check 1: Environment variables
    print("\n1. Environment Variables:")
    print(f"   DATABASE_URL: {'✓ Set' if settings.DATABASE_URL else '✗ Not set'}")
    print(f"   QDRANT_URL: {'✓ Set' if settings.QDRANT_URL else '✗ Not set'}")
    print(f"   OPENAI_API_KEY: {'✓ Set' if settings.OPENAI_API_KEY else '✗ Not set'}")

    if not all([settings.DATABASE_URL, settings.QDRANT_URL, settings.OPENAI_API_KEY]):
        all_checks_passed = False

    # Check 2: Database connection
    print("\n2. Database Connection:")
    storage_service = StorageService()
    try:
        db_healthy = await storage_service.check_database_health()
        print(f"   Postgres: {'✓ Connected' if db_healthy else '✗ Failed'}")
        if not db_healthy:
            all_checks_passed = False
    except Exception as e:
        print(f"   Postgres: ✗ Error - {e}")
        all_checks_passed = False

    # Check 3: Qdrant connection
    print("\n3. Qdrant Connection:")
    try:
        qdrant_healthy = await storage_service.check_qdrant_health()
        print(f"   Qdrant: {'✓ Connected' if qdrant_healthy else '✗ Failed'}")
        if not qdrant_healthy:
            all_checks_passed = False
    except Exception as e:
        print(f"   Qdrant: ✗ Error - {e}")
        all_checks_passed = False

    # Check 4: Project structure
    print("\n4. Project Structure:")
    project_root = Path(__file__).parent.parent.parent
    required_dirs = [
        project_root / "docs",
        project_root / "backend" / "app",
        project_root / "backend" / "tests",
    ]

    for dir_path in required_dirs:
        exists = dir_path.exists()
        print(f"   {dir_path.name}: {'✓ Exists' if exists else '✗ Missing'}")
        if not exists:
            all_checks_passed = False

    # Summary
    print("\n" + "=" * 60)
    if all_checks_passed:
        print("✓ All checks passed! System is ready.")
        sys.exit(0)
    else:
        print("✗ Some checks failed. Please fix the issues above.")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(validate_system())
