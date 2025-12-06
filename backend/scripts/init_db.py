"""Database initialization script

Initializes the hybrid storage system:
1. Tests database connection
2. Runs Alembic migrations
3. Creates Qdrant collection with indexes
4. Verifies all components are healthy
"""

import asyncio
import sys
import os
from pathlib import Path
import logging

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.core.config import settings
from app.services.storage import storage_service
from sqlalchemy import text

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


async def test_database_connection():
    """Test database connectivity"""
    logger.info("Testing database connection...")
    logger.info(f"Database URL: {settings.DATABASE_URL[:50]}...")

    try:
        is_healthy = await storage_service.check_database_health()
        if is_healthy:
            logger.info("✓ Database connection successful")
            return True
        else:
            logger.error("✗ Database connection failed")
            return False
    except Exception as e:
        logger.error(f"✗ Database connection error: {e}")
        return False


async def run_migrations():
    """Run Alembic migrations to create schema"""
    logger.info("\nRunning database migrations...")

    try:
        # Import alembic at runtime to avoid circular dependencies
        from alembic.config import Config
        from alembic import command

        # Get project root and migrations directory
        backend_dir = Path(__file__).parent.parent
        migrations_dir = backend_dir / "migrations"
        alembic_ini = backend_dir / "alembic.ini"

        if not alembic_ini.exists():
            logger.warning("alembic.ini not found - using Alembic programmatically")

            # Create alembic config programmatically
            alembic_cfg = Config()
            alembic_cfg.set_main_option("script_location", str(migrations_dir))
            alembic_cfg.set_main_option("sqlalchemy.url", settings.DATABASE_URL)
        else:
            alembic_cfg = Config(str(alembic_ini))

        # Run migration to head
        command.upgrade(alembic_cfg, "head")

        logger.info("✓ Migrations completed successfully")
        return True

    except Exception as e:
        logger.error(f"✗ Migration error: {e}")
        logger.info("Attempting to create tables manually...")

        try:
            # Fallback: Create tables using raw SQL from migration file
            async with storage_service.get_session() as session:
                # Read and execute migration SQL
                migration_file = Path(__file__).parent.parent / "migrations" / "versions" / "001_initial_schema.py"

                if migration_file.exists():
                    logger.info("Running migration SQL from 001_initial_schema.py...")

                    # Import the migration module
                    import importlib.util
                    spec = importlib.util.spec_from_file_location("migration", migration_file)
                    migration_module = importlib.util.module_from_spec(spec)
                    spec.loader.exec_module(migration_module)

                    # Execute upgrade
                    from alembic.runtime.migration import MigrationContext
                    from alembic.operations import Operations

                    async with storage_service.engine.begin() as conn:
                        await conn.run_sync(lambda sync_conn: migration_module.upgrade())

                    logger.info("✓ Tables created via direct migration execution")
                    return True
                else:
                    logger.error("Migration file not found")
                    return False

        except Exception as fallback_error:
            logger.error(f"✗ Fallback table creation failed: {fallback_error}")
            return False


async def create_qdrant_collection():
    """Create Qdrant collection with proper configuration"""
    logger.info("\nSetting up Qdrant collection...")
    logger.info(f"Qdrant URL: {settings.QDRANT_URL}")

    try:
        # Test Qdrant connection
        is_healthy = await storage_service.check_qdrant_health()
        if not is_healthy:
            logger.error("✗ Qdrant connection failed")
            return False

        logger.info("✓ Qdrant connection successful")

        # Create collection
        success = await storage_service.create_qdrant_collection()
        if success:
            logger.info(f"✓ Qdrant collection '{settings.QDRANT_COLLECTION_NAME}' ready")
            logger.info(f"  - Vector size: {settings.QDRANT_VECTOR_SIZE}")
            logger.info(f"  - Distance metric: COSINE")
            logger.info(f"  - HNSW parameters: m={settings.QDRANT_HNSW_M}, ef_construct={settings.QDRANT_HNSW_EF_CONSTRUCT}")
            logger.info("  - Payload indexes: module, chunk_type, learning_level")
            return True
        else:
            logger.error("✗ Qdrant collection creation failed")
            return False

    except Exception as e:
        logger.error(f"✗ Qdrant setup error: {e}")
        return False


async def verify_schema():
    """Verify all tables and indexes are created"""
    logger.info("\nVerifying database schema...")

    try:
        async with storage_service.get_session() as session:
            # Check for all required tables
            result = await session.execute(text("""
                SELECT table_name
                FROM information_schema.tables
                WHERE table_schema = 'public'
                ORDER BY table_name;
            """))
            tables = [row[0] for row in result.fetchall()]

            expected_tables = ['documents', 'chunks', 'user_queries', 'chunk_usage']
            missing_tables = [t for t in expected_tables if t not in tables]

            if missing_tables:
                logger.error(f"✗ Missing tables: {missing_tables}")
                return False

            logger.info(f"✓ All required tables exist: {', '.join(tables)}")

            # Check for indexes
            result = await session.execute(text("""
                SELECT indexname
                FROM pg_indexes
                WHERE schemaname = 'public'
                ORDER BY indexname;
            """))
            indexes = [row[0] for row in result.fetchall()]
            logger.info(f"✓ Created {len(indexes)} indexes")

            # Check for triggers
            result = await session.execute(text("""
                SELECT trigger_name
                FROM information_schema.triggers
                WHERE trigger_schema = 'public'
                ORDER BY trigger_name;
            """))
            triggers = [row[0] for row in result.fetchall()]
            logger.info(f"✓ Created {len(triggers)} triggers")

            return True

    except Exception as e:
        logger.error(f"✗ Schema verification failed: {e}")
        return False


async def print_status_report():
    """Print final status report"""
    logger.info("\n" + "="*60)
    logger.info("INITIALIZATION STATUS REPORT")
    logger.info("="*60)

    # Database status
    db_healthy = await storage_service.check_database_health()
    logger.info(f"\nPostgres Database: {'✓ HEALTHY' if db_healthy else '✗ UNHEALTHY'}")
    if db_healthy:
        async with storage_service.get_session() as session:
            result = await session.execute(text("SELECT version();"))
            version = result.scalar()
            logger.info(f"  Version: {version.split(',')[0]}")

    # Qdrant status
    qdrant_healthy = await storage_service.check_qdrant_health()
    logger.info(f"\nQdrant Vector DB: {'✓ HEALTHY' if qdrant_healthy else '✗ UNHEALTHY'}")
    if qdrant_healthy:
        collections = storage_service.qdrant_client.get_collections()
        logger.info(f"  Collections: {len(collections.collections)}")
        for col in collections.collections:
            if col.name == settings.QDRANT_COLLECTION_NAME:
                logger.info(f"  - {col.name} (vectors: {col.vectors_count})")

    logger.info("\n" + "="*60)
    logger.info("Initialization complete!")
    logger.info("="*60)


async def main():
    """Main initialization workflow"""
    logger.info("="*60)
    logger.info("Physical AI RAG - Hybrid Storage Initialization")
    logger.info("="*60)
    logger.info(f"Environment: {settings.ENVIRONMENT}")

    try:
        # Step 1: Test database connection
        if not await test_database_connection():
            logger.error("\nFailed to connect to database. Check DATABASE_URL in .env")
            sys.exit(1)

        # Step 2: Run migrations
        if not await run_migrations():
            logger.error("\nFailed to run migrations. Check migration files")
            sys.exit(1)

        # Step 3: Verify schema
        if not await verify_schema():
            logger.error("\nSchema verification failed")
            sys.exit(1)

        # Step 4: Create Qdrant collection
        if not await create_qdrant_collection():
            logger.error("\nFailed to create Qdrant collection. Check QDRANT_URL and QDRANT_API_KEY")
            sys.exit(1)

        # Step 5: Print status report
        await print_status_report()

        logger.info("\n✓ All systems ready! You can now ingest documents.")

    except Exception as e:
        logger.error(f"\n✗ Initialization failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        await storage_service.close()


if __name__ == "__main__":
    asyncio.run(main())
