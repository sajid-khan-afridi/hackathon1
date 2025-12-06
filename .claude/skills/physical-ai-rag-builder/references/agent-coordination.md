# Agent Coordination Patterns

## Table of Contents
- [Agent Architecture](#agent-architecture)
- [Communication Protocols](#communication-protocols)
- [Execution Patterns](#execution-patterns)
- [Error Handling](#error-handling)
- [State Management](#state-management)

## Agent Architecture

### Agent Hierarchy

```
physical-ai-rag-builder (Main Skill/Orchestrator)
    │
    ├─► docusaurus-setup (Agent)
    ├─► project-structure-organizer (Agent)
    ├─► backend-architecture (Agent)
    ├─► hybrid-storage-setup (Agent)
    ├─► rag-pipeline-builder (Agent)
    ├─► integration-layer (Agent)
    └─► testing-validation (Agent)
```

### Agent Responsibilities

#### Main Skill (Orchestrator)
- Analyze user requirements
- Make architectural decisions
- Coordinate agent execution (sequence/parallel)
- Validate outputs from agents
- Ensure cross-agent consistency
- Handle errors and retries

#### Individual Agents
- Execute specific, well-defined tasks
- Report status and results
- Handle task-specific errors
- Request clarification when needed
- Produce verifiable outputs

## Communication Protocols

### Agent Input/Output Contract

```typescript
interface AgentTask {
    agent_name: string;
    task_description: string;
    dependencies: string[];  // Other agents that must complete first
    inputs: {
        context: Record<string, any>;
        requirements: string[];
        constraints: string[];
    };
    outputs_expected: {
        files_created: string[];
        files_modified: string[];
        validation_checklist: string[];
    };
}

interface AgentResult {
    agent_name: string;
    status: "success" | "failed" | "blocked";
    outputs: {
        files_created: string[];
        files_modified: string[];
        artifacts: Record<string, any>;
    };
    validation_results: {
        checklist_item: string;
        passed: boolean;
        details?: string;
    }[];
    next_steps?: string[];
    blockers?: string[];
}
```

### Context Sharing

```python
# Shared context object passed between agents
shared_context = {
    "project": {
        "name": "physical-ai-book",
        "root_dir": "/path/to/project",
        "tech_stack": {
            "frontend": "docusaurus",
            "backend": "fastapi",
            "database": "neon-postgres",
            "vector_db": "qdrant",
            "llm": "openai"
        }
    },
    "docusaurus_setup": {
        "site_url": "http://localhost:3000",
        "build_dir": "build",
        "docs_dir": "docs",
        "config_file": "docusaurus.config.ts"
    },
    "backend_setup": {
        "api_url": "http://localhost:8000",
        "app_dir": "backend/app",
        "main_file": "backend/main.py",
        "requirements_file": "backend/requirements.txt"
    },
    "storage_config": {
        "postgres_url": "postgresql://...",
        "qdrant_url": "https://...",
        "collections": ["physical_ai_chunks"]
    }
}
```

## Execution Patterns

### Pattern 1: Sequential Execution

```python
# When agents depend on each other's outputs
execution_plan = [
    # Phase 1: Architecture planning
    {
        "phase": "planning",
        "agents": ["backend-architecture"],
        "mode": "sequential"
    },
    # Phase 2: Foundation setup (can run in parallel)
    {
        "phase": "foundation",
        "agents": ["docusaurus-setup", "project-structure-organizer"],
        "mode": "parallel"
    },
    # Phase 3: Backend implementation (sequential after foundation)
    {
        "phase": "backend",
        "agents": ["hybrid-storage-setup", "rag-pipeline-builder"],
        "mode": "sequential",
        "depends_on": ["foundation"]
    },
    # Phase 4: Integration (after backend ready)
    {
        "phase": "integration",
        "agents": ["integration-layer"],
        "mode": "sequential",
        "depends_on": ["backend", "foundation"]
    },
    # Phase 5: Testing (final validation)
    {
        "phase": "validation",
        "agents": ["testing-validation"],
        "mode": "sequential",
        "depends_on": ["integration"]
    }
]
```

### Pattern 2: Parallel Execution with Synchronization

```python
async def execute_parallel_phase(agents: list[str], context: dict):
    """Execute multiple agents in parallel, wait for all to complete"""
    tasks = []

    for agent in agents:
        task = launch_agent(agent, context)
        tasks.append(task)

    # Wait for all agents to complete
    results = await asyncio.gather(*tasks, return_exceptions=True)

    # Validate all succeeded
    for agent, result in zip(agents, results):
        if isinstance(result, Exception):
            raise AgentExecutionError(f"{agent} failed: {result}")
        if result.status != "success":
            raise AgentExecutionError(f"{agent} status: {result.status}")

    # Merge outputs into shared context
    for result in results:
        context.update(result.outputs.get("context_updates", {}))

    return context
```

### Pattern 3: Conditional Execution

```python
def plan_execution(user_requirements: dict):
    """Dynamically plan agent execution based on requirements"""

    agents_to_run = []

    # Always run core agents
    agents_to_run.extend(["backend-architecture", "project-structure-organizer"])

    # Conditional: only if Docusaurus doesn't exist
    if not docusaurus_exists():
        agents_to_run.append("docusaurus-setup")

    # Conditional: only if storage not configured
    if not storage_configured():
        agents_to_run.append("hybrid-storage-setup")

    # Always run RAG pipeline and integration
    agents_to_run.extend(["rag-pipeline-builder", "integration-layer"])

    # Conditional: only if user requested tests
    if user_requirements.get("include_tests", True):
        agents_to_run.append("testing-validation")

    return agents_to_run
```

## Error Handling

### Retry Strategy

```python
async def execute_agent_with_retry(
    agent_name: str,
    context: dict,
    max_retries: int = 3
):
    """Execute agent with exponential backoff retry"""

    for attempt in range(max_retries):
        try:
            result = await launch_agent(agent_name, context)

            if result.status == "success":
                return result

            elif result.status == "blocked":
                # Agent needs user input
                user_input = await request_user_clarification(result.blockers)
                context.update(user_input)
                continue

            elif result.status == "failed":
                if attempt < max_retries - 1:
                    # Retry with backoff
                    await asyncio.sleep(2 ** attempt)
                    continue
                else:
                    raise AgentExecutionError(
                        f"{agent_name} failed after {max_retries} attempts"
                    )

        except Exception as e:
            if attempt < max_retries - 1:
                await asyncio.sleep(2 ** attempt)
                continue
            else:
                raise

    raise AgentExecutionError(f"{agent_name} exhausted all retries")
```

### Rollback Strategy

```python
class AgentTransaction:
    """Track agent actions for potential rollback"""

    def __init__(self):
        self.actions = []

    def record_action(self, action_type: str, details: dict):
        self.actions.append({
            "type": action_type,
            "details": details,
            "timestamp": datetime.now()
        })

    async def rollback(self):
        """Reverse all actions in reverse order"""
        for action in reversed(self.actions):
            if action["type"] == "file_created":
                os.remove(action["details"]["path"])
            elif action["type"] == "file_modified":
                # Restore from backup
                restore_file(action["details"]["path"], action["details"]["backup"])
            elif action["type"] == "db_insert":
                # Delete from database
                await db.execute("DELETE FROM ... WHERE id = $1", action["details"]["id"])
```

## State Management

### State Persistence

```python
# Save execution state for resume capability
execution_state = {
    "session_id": "uuid",
    "started_at": "2025-01-15T10:00:00Z",
    "current_phase": "backend",
    "completed_agents": ["backend-architecture", "docusaurus-setup"],
    "failed_agents": [],
    "blocked_agents": [],
    "context": shared_context,
    "checkpoints": [
        {
            "phase": "foundation",
            "completed_at": "2025-01-15T10:15:00Z",
            "context_snapshot": {...}
        }
    ]
}

# Save to disk
with open(".claude/state/execution-state.json", "w") as f:
    json.dump(execution_state, f, indent=2)
```

### Checkpoint and Resume

```python
async def checkpoint_phase(phase_name: str, context: dict):
    """Save checkpoint after completing a phase"""
    checkpoint = {
        "phase": phase_name,
        "completed_at": datetime.now().isoformat(),
        "context_snapshot": copy.deepcopy(context)
    }

    state = load_execution_state()
    state["checkpoints"].append(checkpoint)
    state["current_phase"] = get_next_phase(phase_name)
    save_execution_state(state)

async def resume_from_checkpoint(checkpoint_index: int = -1):
    """Resume execution from a previous checkpoint"""
    state = load_execution_state()
    checkpoint = state["checkpoints"][checkpoint_index]

    context = checkpoint["context_snapshot"]
    next_phase = get_phase_after(checkpoint["phase"])

    return await execute_from_phase(next_phase, context)
```

### Progress Tracking

```python
def report_progress(phase: str, agent: str, progress: float):
    """Report execution progress to user"""
    total_agents = count_total_agents()
    completed_agents = count_completed_agents()

    overall_progress = (completed_agents / total_agents) * 100

    print(f"""
    Phase: {phase}
    Current Agent: {agent} ({progress:.0f}% complete)
    Overall Progress: {overall_progress:.0f}%
    """)
```

## Agent Handoff Protocol

### Clean Handoff

```python
def handoff_to_next_agent(
    current_agent: str,
    next_agent: str,
    outputs: dict,
    context: dict
):
    """
    Clean handoff from one agent to another

    1. Validate current agent outputs
    2. Update shared context
    3. Prepare inputs for next agent
    4. Launch next agent
    """

    # 1. Validate outputs
    validation_results = validate_agent_outputs(current_agent, outputs)
    if not all(v["passed"] for v in validation_results):
        raise ValidationError(f"{current_agent} outputs failed validation")

    # 2. Update context
    context[current_agent] = outputs

    # 3. Prepare next agent inputs
    next_agent_inputs = prepare_agent_inputs(next_agent, context)

    # 4. Launch next agent
    return launch_agent(next_agent, next_agent_inputs)
```
