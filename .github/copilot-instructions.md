# Copilot Coding Agent Instructions

## Repository Overview

**Defold Native Extension** for high-performance A* pathfinding in games. C++11 library with Lua bindings.
- **Size**: ~4,400 lines C++, 53 files, ~600KB
- **Entry Point**: `graph_pathfinder/src/pathfinder.cpp` (Lua API - 458 lines)
- **Core Library**: Header-only in `graph_pathfinder/include/` (11 modules)
- **Platforms**: Desktop (Linux, macOS, Windows), Mobile (iOS, Android), Web (JS, WASM)

## CRITICAL: NO Local Build System

**This repository has NO traditional build tools.** Defold native extensions are compiled by Defold's cloud build server, not locally.

### DO NOT:
- ❌ Try to compile locally (no make, cmake, gcc commands will work)
- ❌ Search for build scripts (they don't exist)
- ❌ Install compilers or build tools

### Validation: clang-format ONLY
**ALWAYS format before committing:**
```bash
clang-format --style=file -i graph_pathfinder/src/pathfinder.cpp
clang-format --style=file -i graph_pathfinder/include/*.h
```

**Verify (dry run):**
```bash
clang-format --style=file --dry-run graph_pathfinder/src/pathfinder.cpp
```

### No Tests
No test suite exists. `example/` directory shows usage but requires Defold Editor.

## Code Style: STRICT C++11

### Absolute Requirements:
1. **No `auto` keyword** - Explicit types only
2. **No STL** - Use `dmArray<T>` instead of `std::vector`, etc.
3. **No C++14/17/20** - Strictly C++11
4. **No exceptions** - Use `PathStatus` enum for errors
5. **No runtime allocation** - Pre-allocated arrays only

### Formatting:
- 4-space indent, no tabs
- Braces on new lines for functions/classes
- See `.clang-format` for details

### Patterns:
**Error Handling:**
```cpp
pathfinder::PathStatus status;
uint32_t result = pathfinder::path::add_node(pos, &status);
if (status != pathfinder::SUCCESS) { /* handle */ }
```

**Lua Bindings** (in pathfinder.cpp):
Use `DM_LUA_STACK_CHECK(L, N)` for stack balance.

## Repository Structure

### Root Files
- `.clang-format` - Code style config (LLVM-based, 4-space indent)
- `game.project` - Defold project config
- `README.md` - Full API documentation

### Directories
**`graph_pathfinder/`** - Native extension source
- `ext.manifest` - Extension metadata (name: "GraphPathfinder")
- `src/pathfinder.cpp` - Lua API (458 lines)
- `include/pathfinder_*.h` - 11 header-only modules (4,400 lines total)
  - `pathfinder_path.h` (262 lines) - Main A* API
  - `pathfinder_math.h` (700 lines) - Vector math
  - `pathfinder_smooth.h` (531 lines) - Path smoothing
  - `pathfinder_types.h` (147 lines) - Data structures (Vec2, Edge, Node)
  - Others: heap, cache, distance_cache, constants, utils
- `include/dmarray.h`, `dmhashtable.h` - Defold SDK containers (DO NOT MODIFY)
- `lib/arm64-osx/` - Pre-built static libraries (managed by Defold)

**`example/`** - Demo Defold project (cannot run locally)

### Lua API (in pathfinder.cpp)
Exposed as `pathfinder.*`:
- `init(max_nodes, max_edges_per_node, pool_block_size, max_cache_path_length)`
- `add_node(x, y)` → node_id
- `add_edge(from_id, to_id, bidirectional)`
- `find_path(start_id, goal_id, max_path_length)` → path_length, status, status_text, path
- `find_projected_path(x, y, goal_id, max_path_length)` → path_length, status, status_text, entry_point, path
- `move_node(node_id, x, y)`, `remove_node(node_id)`, `remove_edge(...)`, `shutdown()`

Constants: `SUCCESS`, `ERROR_NO_PATH`, `ERROR_START_NODE_INVALID`, etc.

## Key Architecture Details

### Memory Model
- Pre-allocated flat arrays (no runtime allocation)
- `dmArray<T>` instead of `std::vector<T>`
- Object pooling for A* heap nodes

### Defold SDK Dependencies
Provided by build server (not available locally):
- `dmsdk/sdk.h`, `dmsdk/script/script.h`, `dmsdk/dlib/log.h`
- `DM_DECLARE_EXTENSION` macro
- Lua stack API: `luaL_checkint`, `lua_pushinteger`, etc.

### Platform Notes
- macOS/iOS: LTO disabled (Defold uses LLVM 15)
- Other platforms: LTO enabled

## Checklist for Changes

### Before Committing:
1. ✅ Format with clang-format
2. ✅ No `auto`, no STL, no C++14+
3. ✅ Use `PathStatus` for errors
4. ✅ Lua functions use `DM_LUA_STACK_CHECK(L, N)`
5. ✅ Update README.md if API changes

### NO CI Pipeline
No GitHub Actions. Manual validation only.

### Example Workflow:
```bash
# Edit code
vim graph_pathfinder/src/pathfinder.cpp

# Format
clang-format --style=file -i graph_pathfinder/src/pathfinder.cpp

# Verify
clang-format --style=file --dry-run graph_pathfinder/src/pathfinder.cpp

# Commit
git commit -am "Add feature X"
```

## Trust These Instructions
This file was created by comprehensive repository analysis. Trust it rather than searching for non-existent build systems. Search only if info is incomplete/incorrect.
