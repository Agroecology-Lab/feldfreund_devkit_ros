# AI Agent Guidelines

> **For**: AI assistants (Cursor, GitHub Copilot, Codex, etc.)\
> **About**: The project, setup and usage is described in [README.md](README.md)\
> **Standards**: All coding standards are in [CONTRIBUTING.md](CONTRIBUTING.md) – follow those rules\
> **NiceGUI Documentation**: A condensed JSON version of NiceGUI's documentation is available at https://nicegui.io/static/sitewide_index.json

## Skills Directory

- Project skills live in `skills/`.
- Skills must be model-agnostic and reusable by any coding assistant.
- Each skill should be stored as `skills/<skill-name>/SKILL.md`.
- When a task matches a skill, read and follow that skill before implementing changes.

## Core Principles

### Think from First Principles

Don't settle for the first solution.
Question assumptions and think deeply about the true nature of the problem before implementing.

### Pair Programming Approach

We work together as pair programmers, switching seamlessly between driver and navigator:

- **Requirements first**: Verify requirements are correct before implementing, especially when writing/changing tests
- **Discuss strategy**: Present options and trade-offs when uncertain about approach
- **Step-by-step for large changes**: Break down significant refactorings and get confirmation at each step
- **Challenge assumptions**: If the user makes wrong assumptions or states untrue facts, correct them directly
- It is crucial that the requirements are right before implementing something. If you write/change tests, ask to verify your assumptions.

### Discuss Before Implementing

For significant changes:

- Present the problem and possible approaches
- Discuss trade-offs and implications
- Get confirmation before proceeding with large refactorings
- Work iteratively with feedback at each step

### Simplicity First

- Prefer simple, straightforward solutions
- Avoid over-engineering
- Remove obsolete code rather than working around it
- Code should be self-explanatory

## Code Organization

- **High-level code first**: Put interesting logic at the top of files
- **Helpers below usage**: Functions called from high-level code should be close to, but below, their usage
- **Keep files focused**: Aim for under 200-300 lines per file; refactor when larger

## What to Avoid

- **Global mutable state** without clear justification
- **Blocking I/O** in async code paths
- **Broad exception catching** without proper error context
- **Debug prints** - use proper logging or remove before committing
- **Unnecessary complexity** - don't introduce new patterns without exhausting existing options
- **Code duplication** - check for similar functionality before implementing
- **Unrelated changes** - stay focused on the requested task

## Quick Verification

Before claiming a task complete, verify:

1. Tests written and passing?
2. Code follows style guidelines?
3. No blocking operations in async code?
4. Debug code removed?
5. Linters passing?

## When Uncertain

- **Check online sources** for inspiration or verification rather than guessing
- **Search the codebase** for similar patterns before inventing new ones
- **Ask the user** by presenting options and trade-offs if strategy is unclear

---

## Code Review Guidelines

**Purpose**: Maximize signal/noise, maintain code quality, and offload maintainers.
Act as a _single, concise reviewer_.
Prefer one structured top-level comment with suggested diffs over many line-by-line nits.

**Standards Reference**: Before starting a review, internalize all coding standards, style guidelines, and contribution workflows defined in [CONTRIBUTING.md](CONTRIBUTING.md) and the principles above.

### Scope & Tone

- Audience: PR authors and maintainers
- Voice: concise, technical, actionable. No style opinions when linters/formatters are green
- Output format: one summary + grouped findings (**BLOCKER**, **MAJOR**, **CLEANUP**) + **suggested diff** blocks where possible

### Severity Mapping

#### BLOCKER (if violated ⇒ request changes)

1. **Security/Secrets**: leaked credentials/keys, unsafe eval/exec, command injection, path traversal, template injection
2. **Concurrency/Async correctness**: event loop blocking (long CPU/I/O in async handlers), missing awaits, race conditions, using `asyncio.create_task()` instead of `background_tasks.create()`, non-thread-safe mutations
3. **Breaking changes**: changes that break existing functionality without clear migration path or deprecation notice
4. **Performance regressions**: O(n²) additions, synchronous I/O in hot paths, unnecessary heavyweight objects
5. **Tests & CI**: missing or incomplete tests; ignoring configured linters/type checks (see [CONTRIBUTING.md](CONTRIBUTING.md))
6. **PR description quality**: missing/vague problem statement or motivation
7. **Formatting & placement**: unformatted files (violates [CONTRIBUTING.md](CONTRIBUTING.md) requirements), surprising file placement without rationale

#### MAJOR (should be fixed before merge)

1. **Error-handling gaps**: exceptions swallowed, broad `except:` clauses, unvalidated user input
2. **File/feature placement**: unexpected location or architecture drift without justification
3. **Unnecessary complexity**: simpler design meets requirements (violates "prefer simple solutions" principle)
4. **Resource hygiene**: unclosed files/sockets/tasks; memory leaks; missing context managers
5. **Logging/observability**: noisy logs, missing error context; debug prints left in code
6. **Cross-platform pitfalls**: Windows paths, locale/timezone assumptions, reliance on system binaries without guards

#### CLEANUP (suggest quick diffs)

1. **Readability**: complex logic without comments; magic numbers; missing docstrings
2. **Test coverage**: edge cases untested (empty/None, large payloads, error conditions)
3. **Micro-optimizations**: tiny allocations in tight loops; missing caching of pure results

### Review Structure

Structure your review comment like this:

**Summary**

Lead with the motivation of the change, then explain what changed and why.
Finish with general risk assessment and impact.

**BLOCKER**

Itemized violations of critical rules with short rationale

**MAJOR**

Concrete issues that should be fixed pre-merge

**CLEANUP**

Low-noise, quick-win improvements

**Suggested diffs**

Use diff/suggestion blocks (apply only if trivial and safe):

```diff
- data = open('config.json').read()
+ with open('config.json') as f:
+     data = f.read()
```

### Review Behavior

- Prefer **one** top-comment; avoid scatter
- If evidence is weak/speculative, ask a short question instead of asserting
- If change is broad: propose a tiny follow-up PR rather than expanding this one

### Review Checklist

Mental checklist before posting review:

1. **Async paths non-blocking?** Blocking operations properly handled?
2. **Tests added/updated?** Coverage for edge cases? No flakiness?
3. **Code follows [CONTRIBUTING.md](CONTRIBUTING.md)?** Style, organization, principles?
4. **Documentation updated?** If behavior changed, are docs/examples updated?
5. **Security basics ok?** Inputs validated? No dangerous operations? Secrets handled properly?
6. **Breaking changes?** Backward compatibility preserved or migration path clear?

---

> This file complements [CONTRIBUTING.md](CONTRIBUTING.md).
> For detailed workflow, testing, and contribution process, see CONTRIBUTING.md.
>
> Maintainers: update this file as conventions evolve.
> If changes are general, consider creating a PR to https://github.com/zauberzeug/nicegui-template so others can benefit from it.
