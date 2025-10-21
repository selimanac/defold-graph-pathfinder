# Copilot Coding Agent Instructions

## Repository Overview

**Defold Native Extension** for high-performance A* pathfinding in games. C++11 library with Lua bindings.
- **Size**: ~5,200 lines C++, 32 files, ~700KB
- **Entry Points**: 
  - `graph_pathfinder/src/pathfinder.cpp` (Lua API - 904 lines)
  - `graph_pathfinder/src/pathfinder_extension.cpp` (Extension core - 333 lines)
- **Core Library**: Header-only in `graph_pathfinder/include/` (19 modules)
- **Documentation**: 
  - `API.md` - Complete API reference
  - `graph_pathfinder/annotations.lua` - Lua type annotations for IDE support
- **Platforms**: Desktop (Linux, macOS, Windows), Mobile (iOS, Android), Web (JS, WASM)

## CRITICAL: NO Local Build System

**This repository has NO traditional build tools.** Defold native extensions are compiled by Defold's cloud build server, not locally.

### DO NOT:
- ❌ Try to compile locally (no make, cmake, gcc commands will work)
- ❌ Search for build scripts (they don't exist)
- ❌ Install compilers or build tools

### CI/CD with GitHub Actions (Optional)
This repository can use Defold's reusable workflows for automated builds:

**Bob Build Workflow** (from [defold/github-actions-common](https://github.com/defold/github-actions-common)):
```yaml
name: Build with bob
on: [push, pull_request]
jobs:
  build:
    uses: defold/github-actions-common/.github/workflows/bob.yml@master
```

Create this as `.github/workflows/build.yml` to enable automated builds on push/PR.

### Validation: clang-format
**ALWAYS format before committing:**
```bash
clang-format --style=file -i graph_pathfinder/src/pathfinder.cpp
clang-format --style=file -i graph_pathfinder/src/pathfinder_extension.cpp
clang-format --style=file -i graph_pathfinder/include/*.h
```

**Verify (dry run):**
```bash
clang-format --style=file --dry-run graph_pathfinder/src/pathfinder.cpp
clang-format --style=file --dry-run graph_pathfinder/src/pathfinder_extension.cpp
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

## New Features & Files (Recent Additions)

### annotations.lua
**Purpose:** Lua Language Server (LLS) type annotations for IDE support
**Location:** `graph_pathfinder/annotations.lua`
**Features:**
- Complete type definitions for all API functions
- `@class` definitions: PathNode, PathEdge, PathSmoothConfig, GameObjectNodeConfig, CacheStats
- `@enum` definitions: PathStatus, PathSmoothStyle
- Enables autocomplete and type checking in IDEs like VS Code with Lua extension

**Usage:** No runtime impact - purely for IDE/editor support

### API.md
**Purpose:** Comprehensive API documentation
**Location:** Root directory `API.md`
**Content:**
- Detailed function signatures with parameter descriptions
- Return value documentation
- Code examples for each function
- Enumeration tables with descriptions
- Data type specifications
- Complete working example
- Performance tips

**When to Update:** Add or modify entries when changing the Lua API

### pathfinder_extension.cpp/.h
**Purpose:** Core extension functionality separate from Lua bindings
**Location:** 
- `graph_pathfinder/src/pathfinder_extension.cpp` (333 lines)
- `graph_pathfinder/include/pathfinder_extension.h` (45 lines)

**Responsibilities:**
1. **Game Object Node Management:**
   - Hash table storage for game object tracking
   - Automatic position synchronization
   - Pause/resume individual nodes
   - World vs local position support

2. **Update Loop:**
   - Fixed or variable timestep
   - Configurable frequency (Hz)
   - Iterates active game objects
   - Updates graph node positions

3. **Smoothing Configuration:**
   - Stores up to 64 smoothing configs
   - ID-based reference system
   - Combines style enum + parameters

4. **Cache Statistics:**
   - Aggregates path cache metrics
   - Aggregates distance cache metrics
   - Exposes to Lua via get_cache_stats()

**Key Functions:**
- `init()` / `shutdown()` - Lifecycle
- `set_gameobject_capacity()` - Pre-allocate GO storage
- `add_gameobject_node()` / `remove_gameobject_node()`
- `pause_gameobject_node()` / `resume_gameobject_node()`
- `set_update_state()` / `set_update_frequency()`
- `update()` - Called every frame
- `add_smooth_config()` - Create smoothing config
- `smooth_path()` / `smooth_path_waypoint()` - Apply smoothing
- `get_cache_stats()` - Retrieve cache metrics

**Integration:**
- Called from `pathfinder.cpp` Lua bindings
- Registered via `DM_DECLARE_EXTENSION` lifecycle hooks:
  - `AppInitializeGraphPathfinder` - Sets up extension on app start
  - `OnUpdateGraphPathfinder` - Called every frame
  - `AppFinalizeGraphPathfinder` - Cleanup on app shutdown

### New Lua API Functions

**Batch Operations:**
- `add_nodes(node_positions)` - Create multiple nodes at once
- `add_edges(edges)` - Create multiple edges at once
- `add_gameobject_nodes(game_object_nodes)` - Create multiple GO nodes

**Game Object Nodes:**
- `add_gameobject_node()` - Single GO node with auto-update
- `add_gameobject_nodes()` - Batch GO nodes
- `convert_gameobject_node()` - Convert existing node to GO node
- `remove_gameobject_node()` - Remove GO node
- `pause_gameobject_node()` - Pause position updates
- `resume_gameobject_node()` - Resume position updates
- `gameobject_update(enabled)` - Global enable/disable
- `set_update_frequency(hz)` - Set update rate

**Smoothing:**
- `add_path_smoothing(config)` - Create smoothing configuration
- `smooth_path(smooth_id, waypoints)` - Apply smoothing to waypoints
- Note: `find_path()` and `find_projected_path()` now accept optional `smooth_id` parameter

**Statistics:**
- `get_cache_stats()` - Returns detailed cache metrics

**Enhanced Functions:**
- `init()` - Now accepts `max_gameobject_nodes` parameter
- `add_edge()` - Now accepts optional `cost` parameter
- `get_node_position()` - Returns position table

## Repository Structure

### Root Files
- `.clang-format` - Code style config (LLVM-based, 4-space indent)
- `game.project` - Defold project config
- `README.md` - Feature overview and getting started guide
- `API.md` - Complete API reference with examples and data types

### Directories
**`graph_pathfinder/`** - Native extension source
- `ext.manifest` - Extension metadata (name: "GraphPathfinder")
- `annotations.lua` - Lua type annotations for IDE support (207 lines)
- `src/pathfinder.cpp` - Lua API bindings (904 lines)
- `src/pathfinder_extension.cpp` - Extension core functionality (333 lines)
  - Game object node tracking and automatic updates
  - Path smoothing configuration management
  - Cache statistics aggregation
  - Update loop with configurable frequency
- `include/pathfinder_extension.h` - Extension interface (45 lines)
- `include/pathfinder_*.h` - 11 pathfinding modules (4,400 lines total)
  - `pathfinder_path.h` (262 lines) - Main A* API
  - `pathfinder_math.h` (701 lines) - Vector math
  - `pathfinder_smooth.h` (532 lines) - Path smoothing
  - `pathfinder_types.h` (148 lines) - Data structures (Vec2, Edge, Node)
  - `pathfinder_cache.h` - Path result caching
  - `pathfinder_distance_cache.h` - Heuristic caching
  - Others: heap, constants, utils
- `include/navigation_*.h` - 8 navigation system modules
  - `navigation_types.h` - Core types
  - `navigation_path.h` - Path management
  - `navigation_agents.h` - Agent behaviors
  - `navigation_path_storage.h` - Path storage
  - `navigation_constants.h` - System constants
- `include/dmarray.h`, `dmhashtable.h` - Defold SDK containers (DO NOT MODIFY)
- `lib/arm64-osx/` - Pre-built static libraries (managed by Defold)

**`example/`** - Demo Defold project (cannot run locally)

### Lua API (in pathfinder.cpp)
Exposed as `pathfinder.*`:

**Initialization:**
- `init(max_nodes, max_gameobject_nodes, max_edges_per_node, pool_block_size, max_cache_path_length)`
- `shutdown()`

**Node Management:**
- `add_node(x, y)` → node_id
- `add_nodes(node_positions)` → node_ids (batch operation)
- `remove_node(node_id)`
- `move_node(node_id, x, y)`
- `get_node_position(node_id)` → position

**Edge Management:**
- `add_edge(from_id, to_id, bidirectional, cost)`
- `add_edges(edges)` → (batch operation)
- `remove_edge(from_id, to_id, bidirectional)`

**Pathfinding:**
- `find_path(start_id, goal_id, max_path_length, smooth_id)` → path_length, status, status_text, path
- `find_projected_path(x, y, goal_id, max_path_length, smooth_id)` → path_length, status, status_text, entry_point, path

**Path Smoothing:**
- `add_path_smoothing(config)` → smooth_id
- `smooth_path(smooth_id, waypoints)` → smoothed_length, smoothed_path

**Game Object Nodes:**
- `add_gameobject_node(game_object_instance, use_world_position)` → node_id
- `add_gameobject_nodes(game_object_nodes)` → node_ids (batch operation)
- `convert_gameobject_node(node_id, game_object_instance, use_world_position)`
- `remove_gameobject_node(node_id)`
- `pause_gameobject_node(node_id)`
- `resume_gameobject_node(node_id)`

**Update Control:**
- `gameobject_update(enabled)` - Enable/disable automatic GO node updates
- `set_update_frequency(frequency)` - Set update frequency in Hz

**Statistics:**
- `get_cache_stats()` → stats (path_cache and distance_cache metrics)

**Enums:**
- `PathStatus` - SUCCESS, ERROR_NO_PATH, ERROR_START_NODE_INVALID, ERROR_GOAL_NODE_INVALID, ERROR_NODE_FULL, ERROR_EDGE_FULL, ERROR_HEAP_FULL, ERROR_PATH_TOO_LONG, ERROR_GRAPH_CHANGED, ERROR_GRAPH_CHANGED_TOO_OFTEN, ERROR_NO_PROJECTION, ERROR_VIRTUAL_NODE_FAILED
- `PathSmoothStyle` - NONE, CATMULL_ROM, BEZIER_CUBIC, BEZIER_QUADRATIC, BEZIER_ADAPTIVE, CIRCULAR_ARC

## Key Architecture Details

### Memory Model
- Pre-allocated flat arrays (no runtime allocation)
- `dmArray<T>` instead of `std::vector<T>`
- Object pooling for A* heap nodes

### Extension Architecture (`pathfinder_extension.cpp` and `.h`)
**Game Object Node Tracking:**
- Tracks game object positions automatically via update loop
- Hashtable of game objects with pause/resume capability
- Supports both world and local position modes
- Configurable update frequency (synced with display.update_frequency)

**Path Smoothing Management:**
- Stores up to 64 smoothing configurations
- Each configuration has style + parameters
- Configurations referenced by ID in pathfinding calls

**Cache Statistics:**
- Aggregates metrics from path cache and distance cache
- Hit rates, capacity, entries counted
- Useful for performance tuning

**Update Loop:**
- Called on every frame via `OnUpdateGraphPathfinder`
- Variable or fixed timestep based on update_frequency
- Iterates all active (non-paused) game object nodes
- Updates node positions in pathfinding graph

### Defold SDK Dependencies
Provided by build server (not available locally):
- `dmsdk/sdk.h`, `dmsdk/script/script.h`, `dmsdk/dlib/log.h`
- `dmsdk/gameobject/gameobject.h` - For game object tracking
- `dmsdk/dlib/hashtable.h` - For game object storage
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
5. ✅ Update API.md if API changes (not README.md)
6. ✅ Update annotations.lua if adding/modifying Lua functions

### CI Pipeline (Optional)
Currently no GitHub Actions configured. Can add Bob build workflow (see CI/CD section above).

### Example Workflow:
```bash
# Edit code
vim graph_pathfinder/src/pathfinder.cpp

# Format all modified files
clang-format --style=file -i graph_pathfinder/src/pathfinder.cpp
clang-format --style=file -i graph_pathfinder/src/pathfinder_extension.cpp

# Verify formatting
clang-format --style=file --dry-run graph_pathfinder/src/pathfinder.cpp
clang-format --style=file --dry-run graph_pathfinder/src/pathfinder_extension.cpp

# Update documentation if needed
vim API.md
vim graph_pathfinder/annotations.lua

# Commit
git commit -am "Add feature X"
```

## Trust These Instructions
This file was created by comprehensive repository analysis. Trust it rather than searching for non-existent build systems. Search only if info is incomplete/incorrect.
