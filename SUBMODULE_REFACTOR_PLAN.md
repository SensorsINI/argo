# Submodule Refactoring Plan

## Current Structure (Before)
```
argo/
├── simulator/                    # Submodule (currently at root)
│   ├── boats/
│   ├── environments/
│   ├── sailboat_playground/      # Python package
│   └── ...
├── sailboat-playground/          # Local customizations
│   ├── boats/sample_boat.json
│   ├── environments/playground.json
│   └── ...
└── .gitmodules                   # Points to simulator/
```

## Proposed Structure (After)
```
argo/
├── simulator/
│   ├── sailboat-playground/      # Submodule (moved here)
│   │   ├── boats/
│   │   ├── environments/
│   │   ├── sailboat_playground/  # Python package
│   │   └── ...
│   └── customizations/
│       └── sailboat-playground/  # Argo-specific configs
│           ├── boats/sample_boat.json
│           ├── environments/playground.json
│           └── ...
└── .gitmodules                   # Points to simulator/sailboat-playground
```

## Rationale
1. **Clearer organization**: Submodule is nested under `simulator/` parent directory
2. **Explicit customizations**: Custom configs clearly separated in `customizations/` directory
3. **Better isolation**: Source code and config files in different directory trees
4. **Scalability**: Easy to add other simulator-related components

## Files Requiring Updates

### 1. Git Configuration
- `.gitmodules` - Update submodule path from `simulator` to `simulator/sailboat-playground`

### 2. Python Code
- `nodes/argo_unified_simulator_bridge.py`:
  - Line 37: Update simulator path: `simulator/sailboat-playground`
  - Line 124-125: Update config paths: `simulator/customizations/sailboat-playground/...`

### 3. Makefile
- `Makefile` submodule targets:
  - `submodule-init`: Update path to `simulator/sailboat-playground`
  - `submodule-update`: Update path to `simulator/sailboat-playground`
  - `submodule-status`: Update directory checks and messages

### 4. Documentation
- `SIMULATOR_SETUP.md`: Update all references to config paths
- `README.md`: Update simulator directory structure documentation
- `.cursor/rules/argo-submodule-management.mdc`: Update paths and examples
- `.cursor/rules/argo-simulation-local.mdc`: Update configuration paths

### 5. Scripts (No changes needed)
- `scripts/launch_simulator_local.sh` - No changes (uses node directly)
- `scripts/remote_simulator_launch.py` - No changes (uses node directly)
- `scripts/load_config.py` - No changes (no simulator paths)

## Migration Steps

### Step 1: Backup Current State
```bash
# Backup current sailboat-playground config directory
cp -r sailboat-playground sailboat-playground.backup
```

### Step 2: Remove Current Submodule
```bash
# Deinitialize current submodule
git submodule deinit -f simulator
git rm -f simulator
rm -rf .git/modules/simulator
```

### Step 3: Create New Directory Structure
```bash
# Create new simulator directory structure
mkdir -p simulator/customizations
mv sailboat-playground simulator/customizations/sailboat-playground
```

### Step 4: Update Git Configuration
```bash
# Update .gitmodules
# Change path from "simulator" to "simulator/sailboat-playground"
```

### Step 5: Initialize New Submodule
```bash
# Initialize submodule at new location
git submodule add https://github.com/SensorsINI/sailboat-playground.git simulator/sailboat-playground
git submodule update --init --recursive simulator/sailboat-playground
```

### Step 6: Update Python Code
```bash
# Update argo_unified_simulator_bridge.py paths
# - simulator_path = .../simulator/sailboat-playground
# - config paths = simulator/customizations/sailboat-playground/...
```

### Step 7: Update Makefile
```bash
# Update all submodule targets to use new path
```

### Step 8: Update Documentation
```bash
# Update all documentation files with new paths
```

### Step 9: Test
```bash
# Test local simulation
python3 nodes/argo_unified_simulator_bridge.py --mode local

# Test submodule management
make submodule-status
make submodule-update
```

### Step 10: Commit Changes
```bash
git add .gitmodules simulator/ Makefile nodes/ .cursor/
git commit -m "Refactor: Reorganize simulator submodule structure

- Move submodule to simulator/sailboat-playground
- Move customizations to simulator/customizations/sailboat-playground
- Update all references to new paths
- Update Makefile submodule targets
- Update documentation and Cursor rules"
```

## Risk Assessment

### Low Risk
- Scripts in `scripts/` - Don't reference simulator paths directly
- Remote simulation - Uses same bridge node

### Medium Risk
- Makefile targets - Need path updates but well isolated
- Documentation - Multiple files to update

### High Risk
- `argo_unified_simulator_bridge.py` - Core functionality, needs careful testing
- Submodule re-initialization - Must not lose customizations

## Rollback Plan

If issues occur:
```bash
# Restore from backup
git submodule deinit -f simulator/sailboat-playground
git rm -f simulator/sailboat-playground
rm -rf simulator/customizations
cp -r sailboat-playground.backup sailboat-playground

# Restore original submodule
git submodule add https://github.com/SensorsINI/sailboat-playground.git simulator
git submodule update --init --recursive simulator
```

## Validation Checklist

- [ ] Submodule at correct path: `simulator/sailboat-playground/`
- [ ] Customizations at correct path: `simulator/customizations/sailboat-playground/`
- [ ] Python imports work: `import sailboat_playground`
- [ ] Config files load: boat and environment JSON
- [ ] Local simulation works
- [ ] Remote simulation works
- [ ] Makefile targets work: `make submodule-status`
- [ ] Documentation updated
- [ ] All tests pass

