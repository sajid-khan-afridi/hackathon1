.PHONY: install dev test build clean help

# Colors for output
BLUE := \033[0;34m
GREEN := \033[0;32m
YELLOW := \033[0;33m
NC := \033[0m # No Color

help: ## Show this help message
	@echo "$(BLUE)Physical AI RAG System - Development Commands$(NC)"
	@echo ""
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "$(GREEN)%-15s$(NC) %s\n", $$1, $$2}'

install: ## Install all dependencies (frontend + backend)
	@echo "$(BLUE)Installing dependencies...$(NC)"
	@echo "$(YELLOW)Installing Node.js dependencies...$(NC)"
	npm install
	@echo "$(YELLOW)Installing Python dependencies...$(NC)"
	cd backend && pip install -r requirements.txt -r requirements-dev.txt
	@echo "$(GREEN)✓ Dependencies installed$(NC)"

dev: ## Run development servers (frontend + backend)
	@echo "$(BLUE)Starting development servers...$(NC)"
	@echo "$(YELLOW)Starting Docker services...$(NC)"
	docker-compose -f shared/docker/docker-compose.yml up -d postgres qdrant
	@echo "$(YELLOW)Waiting for services to be ready...$(NC)"
	sleep 5
	@echo "$(YELLOW)Starting backend...$(NC)"
	cd backend && uvicorn app.main:app --reload --port 8000 &
	@echo "$(YELLOW)Starting frontend...$(NC)"
	npm start

dev-backend: ## Run only backend development server
	@echo "$(BLUE)Starting backend server...$(NC)"
	docker-compose -f shared/docker/docker-compose.yml up -d postgres qdrant
	sleep 5
	cd backend && uvicorn app.main:app --reload --port 8000

dev-frontend: ## Run only frontend development server
	@echo "$(BLUE)Starting frontend server...$(NC)"
	npm start

docker-up: ## Start all Docker services
	@echo "$(BLUE)Starting Docker services...$(NC)"
	docker-compose -f shared/docker/docker-compose.yml up -d
	@echo "$(GREEN)✓ Docker services started$(NC)"

docker-down: ## Stop all Docker services
	@echo "$(BLUE)Stopping Docker services...$(NC)"
	docker-compose -f shared/docker/docker-compose.yml down
	@echo "$(GREEN)✓ Docker services stopped$(NC)"

docker-logs: ## Show Docker logs
	docker-compose -f shared/docker/docker-compose.yml logs -f

test: ## Run all tests
	@echo "$(BLUE)Running tests...$(NC)"
	@echo "$(YELLOW)Running backend tests...$(NC)"
	cd backend && pytest tests/ -v --cov=app --cov-report=term-missing
	@echo "$(YELLOW)Running frontend tests...$(NC)"
	npm test
	@echo "$(GREEN)✓ Tests complete$(NC)"

test-backend: ## Run only backend tests
	@echo "$(BLUE)Running backend tests...$(NC)"
	cd backend && pytest tests/ -v --cov=app --cov-report=term-missing

test-frontend: ## Run only frontend tests
	@echo "$(BLUE)Running frontend tests...$(NC)"
	npm test

build: ## Build for production
	@echo "$(BLUE)Building for production...$(NC)"
	@echo "$(YELLOW)Building frontend...$(NC)"
	npm run build
	@echo "$(GREEN)✓ Build complete$(NC)"

clean: ## Clean build artifacts and caches
	@echo "$(BLUE)Cleaning build artifacts...$(NC)"
	rm -rf node_modules build .docusaurus
	rm -rf backend/__pycache__ backend/.pytest_cache backend/htmlcov
	find backend -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
	find backend -type f -name "*.pyc" -delete
	@echo "$(GREEN)✓ Cleaned$(NC)"

init-db: ## Initialize database schema
	@echo "$(BLUE)Initializing database...$(NC)"
	cd backend && python scripts/init_db.py
	@echo "$(GREEN)✓ Database initialized$(NC)"

ingest: ## Ingest documents into RAG system
	@echo "$(BLUE)Ingesting documents...$(NC)"
	cd backend && python scripts/ingest_documents.py
	@echo "$(GREEN)✓ Documents ingested$(NC)"

validate: ## Validate system setup
	@echo "$(BLUE)Validating system...$(NC)"
	cd backend && python scripts/validate_setup.py

format: ## Format code (backend only)
	@echo "$(BLUE)Formatting code...$(NC)"
	cd backend && black app/ tests/
	@echo "$(GREEN)✓ Code formatted$(NC)"

lint: ## Lint code (backend only)
	@echo "$(BLUE)Linting code...$(NC)"
	cd backend && flake8 app/ tests/
	cd backend && mypy app/
	@echo "$(GREEN)✓ Linting complete$(NC)"
