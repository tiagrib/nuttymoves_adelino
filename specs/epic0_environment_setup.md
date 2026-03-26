# Epic 0: Environment Setup

**Type**: DevOps / Infrastructure
**Status**: Complete
**Priority**: P0 (Blocking)

---

## Overview

Set up the complete development environment for NuttyMoves on Windows, including Isaac Sim, Isaac Lab, Python environment management, and all dependencies.

## Goals

- Reproducible development environment setup
- Automated setup scripts for new developers
- Verification tools to validate installation
- Clear documentation of all dependencies and paths

## Prerequisites

- Windows 10/11
- NVIDIA GPU with 6GB+ VRAM
- NVIDIA Driver 580+ (recommended)
- Git installed

---

## Tasks

### Task 0.1: Install Isaac Sim via pip
**Status**: Complete (Tested 2026-01-19)

Isaac Sim 5.1.0 is now installed via pip (not Omniverse Launcher):

```powershell
pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
```

**Key Findings**:
- Download size: ~4GB
- Installs to site-packages (no separate ISAACSIM_PATH needed)
- First run downloads ~10GB of extensions
- EULA auto-accepted via `OMNI_KIT_ACCEPT_EULA=yes`

**Acceptance Criteria**: [PASS]
- isaacsim module imports successfully
- Isaac Sim launches and renders

---

### Task 0.2: Install Isaac Lab Extensions
**Status**: Complete (Tested 2026-01-19)

Isaac Lab cloned to `C:\repo\isaaclab` and extensions installed manually.

**Key Findings**:
- `isaaclab.bat --install` does NOT work with pyenv-venv (checks for CONDA_PREFIX)
- Manual pip install required for each extension
- Extensions must be installed in editable mode for development

```powershell
pip install --editable C:\repo\isaaclab\source\isaaclab
pip install --editable C:\repo\isaaclab\source\isaaclab_assets
pip install --editable C:\repo\isaaclab\source\isaaclab_tasks
pip install --editable C:\repo\isaaclab\source\isaaclab_rl
```

**Acceptance Criteria**: [PASS]
- isaaclab module imports successfully
- Isaac Lab tutorials run

---

### Task 0.3: Install PyTorch with CUDA
**Status**: Complete (Tested 2026-01-19)

PyTorch 2.7.0+cu128 installed with CUDA 12.8 support.

**Key Findings**:
- Isaac Sim pip package includes CPU-only PyTorch
- Must reinstall with CUDA index for GPU training
- CUDA 12.8 is the current recommended version

```powershell
pip install -U torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128
```

**Verification**:
```
PyTorch: 2.7.0+cu128
CUDA: Available
Device: Quadro RTX 3000
```

**Acceptance Criteria**: [PASS]
- `torch.cuda.is_available()` returns True
- GPU device detected

---

### Task 0.4: Install pyenv-win and pyenv-win-venv
**Status**: Complete (Pre-existing)

Environment `nuttymoves` created with Python 3.11.11.

**Location**: `C:\Users\tiagr\.pyenv-win-venv\envs\nuttymoves`

**Acceptance Criteria**: [PASS]
- `pyenv-venv activate nuttymoves` works
- Python 3.11.x available

---

### Task 0.5: Create Setup Scripts
**Status**: Complete (Updated 2026-01-19)

Scripts created/updated:
- [x] `scripts/setup.ps1` - Complete rewrite for pip-based installation
- [x] `scripts/verify_setup.py` - Verification script
- [x] `requirements.txt` - Python dependencies

**setup.ps1 Features**:
- Validates prerequisites (Git, pyenv-venv, NVIDIA driver)
- Creates pyenv-venv virtual environment
- Installs Isaac Sim 5.1 via pip
- Installs PyTorch with CUDA support
- Clones and installs Isaac Lab extensions
- Configures environment variables (ISAACLAB_PATH, OMNI_KIT_ACCEPT_EULA)
- Runs verification checks

---

### Task 0.6: Document Environment Setup
**Status**: Complete (Updated 2026-01-19)

Documentation updated:
- [x] `workflow/setup_guide.md` - Comprehensive guide with pip-based approach
- [x] `.claudecode/agent-rules.md` - Agent context for development

---

### Task 0.7: Create Launch Tests
**Status**: Complete (Added 2026-01-20)

Test scripts to verify Isaac Sim and Isaac Lab actually launch:
- [x] `scripts/test_isaacsim.py` - Launches Isaac Sim, creates empty world
- [x] `scripts/test_isaaclab.py` - Launches Isaac Lab cartpole environment

**Usage**:
```powershell
python scripts/test_isaacsim.py   # Test Isaac Sim launches
python scripts/test_isaaclab.py   # Test Isaac Lab with cartpole
```

---

### Task 0.8: Consolidate Scripts
**Status**: Complete (Updated 2026-01-21)

Scripts consolidated to minimal set:
- `scripts/setup.ps1` - Main setup script
- `scripts/verify_setup.py` - Verification script

---

## Environment Variables

| Variable | Purpose | Value |
|----------|---------|-------|
| `ISAACLAB_PATH` | Isaac Lab repository | `C:\repo\isaaclab` |
| `OMNI_KIT_ACCEPT_EULA` | Auto-accept Omniverse EULA | `yes` |

**Note**: ISAACSIM_PATH is no longer needed with pip-based installation.

---

## Files Created/Modified

| File | Purpose |
|------|---------|
| `scripts/setup.ps1` | Main setup script |
| `scripts/verify_setup.py` | Verification script |
| `scripts/test_isaacsim.py` | Launch test for Isaac Sim |
| `scripts/test_isaaclab.py` | Launch test for Isaac Lab |
| `workflow/setup_guide.md` | Setup documentation |

---

## Dependencies

### Python Packages (installed via pip)
- isaacsim[all,extscache]==5.1.0 (from pypi.nvidia.com)
- torch==2.7.0+cu128
- torchvision==0.22.0+cu128
- isaaclab (editable)
- isaaclab_assets (editable)
- isaaclab_tasks (editable)
- isaaclab_rl (editable)

### External Software
- NVIDIA Driver 580+
- Git
- pyenv-win
- pyenv-win-venv

---

## Completion Criteria

- [x] All tasks marked complete
- [x] Setup script tested on existing environment
- [x] Documentation reviewed and updated
- [x] Can successfully import Isaac Lab and Isaac Sim
- [x] PyTorch CUDA verified working

---

## Lessons Learned

1. **pip-based installation is now preferred** - Simpler than Omniverse Launcher
2. **isaaclab.bat doesn't work with pyenv-venv** - Manual pip install required
3. **PyTorch from Isaac Sim is CPU-only** - Must reinstall with CUDA index
4. **pxr module only available at runtime** - Cannot import directly
5. **First run downloads ~10GB** - Warn users about initial setup time
6. **OMNI_KIT_ACCEPT_EULA auto-accepts license** - Essential for automation
