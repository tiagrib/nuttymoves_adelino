# projects/adelino Agent Guide

Repo-specific agent instructions for `projects/adelino`.
Read the root `AGENTS.md` first for global rules, project structure, and coding standards.

## Repo Overview

This repo contains everything Adelino-specific:
- Standalone Rust controller binary (`standalone/`)
- Arduino firmware (`firmware/`)
- Robot embodiment assets (`embodiment/`)
- Isaac Lab RL training (`source/adelino_lab/`)
- Deployment bridge (`deployment/`)
- User guides (`guides/`)

## Agent Rules

1. Follow all rules in the root `AGENTS.md`.
2. **Do not modify `metak-shared/`.** Propose changes via the orchestrator for user review.
3. Read your assignments from `metak-orchestrator/TASKS.md` and update `metak-orchestrator/STATUS.md` when done or blocked.
4. Consult `metak-shared/LEARNED.md` for useful methods, procedures, and tricks discovered during the project. Add new learnings as you discover them.
5. <!-- Add any repo-specific rules here. -->

## Coding Standards

- Follow the coding standards defined in `metak-shared/coding-standards.md` for your repo's language.
- <!-- Add language, framework, and linting conventions specific to this repo. -->

## Custom Instructions

Read and follow `CUSTOM.md` in this directory for repo-specific custom instructions.
