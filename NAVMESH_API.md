# Navmesh Pathfinding API Documentation

Defold Graph Pathfinder Extension - Navigation mesh pathfinding with polygon-based A* and funnel algorithm for smooth, optimal paths through walkable areas.

## Table of Contents

- [Introduction](#introduction)
- [Initialization](#initialization)
- [Configuration](#configuration)
- [Buffer Management](#buffer-management)
- [Pathfinding](#pathfinding)
- [Spatial Index](#spatial-index)
- [Statistics](#statistics)
- [Enumerations](#enumerations)


---

## Introduction

The navmesh pathfinding system provides polygon-based pathfinding for games requiring smooth character movement through complex environments. It implements:

- **Polygon-based A\*** algorithm for finding cell corridors through the navigation mesh
- **Simple Stupid Funnel Algorithm (SSFA)** for extracting optimal smooth paths
- **Agent radius support** for runtime collision avoidance via portal offsetting
- **Spatial grid indexing** for fast cell lookup at arbitrary positions
- **LRU path caching** with automatic invalidation on mesh changes

**Key Features:**
- Handles arbitrary convex polygon cells (not just triangles)
- Automatic adjacency detection via edge hashing
- Fallback positioning for clicks outside walkable areas
- Cache provides 10-100× speedup for repeated queries
- Zero runtime allocation after initialization

**Performance:**
- Pathfinding: O((C + E) log C + P) where C=cells, E=edges, P=portals
- Cell lookup: O(1) average with spatial index
- Memory: O(max_cells × (vertices + neighbors) + spatial_grid_size)

---

## Initialization

### pathfinder.navmesh_init()

Initialize the navigation mesh pathfinding system. Must be called before any other navmesh operations.

**Syntax:**
```lua
pathfinder.navmesh_init(max_cells, max_edges_per_cell, pool_block_size, cache_size, max_cache_path_length, min_cell_size, max_cell_size, max_grid_dim, debug)
```

**Parameters:**
- `max_cells` (number): Maximum number of polygon cells in the navigation mesh
- `max_edges_per_cell` (number): Maximum edges/neighbors per cell (typically 3-8)
- `pool_block_size` (number): Heap pool block size for A* algorithm (default: 32)
- `cache_size` (number): Number of paths to cache (0 to disable, recommended: 16-128)
- `max_cache_path_length` (number): Maximum length of cached paths in cells (default: 256)
- `min_cell_size` (number)[optional, default: 5.0]: Minimum spatial index grid cell size
- `max_cell_size` (number)[optional, default: 10.0]: Maximum spatial index grid cell size
- `max_grid_dim` (number)[optional, default: 1000]: Maximum spatial index grid dimension
- `debug` (boolean)[optional, default: false]: Enable debug output (requires NAVMESH_DEBUG=1 at compile time)

> [!IMPORTANT]
> The heap pool capacity equals `max_cells`. If `pool_block_size > max_cells`, it will be automatically clamped to `max_cells` to prevent heap allocation failures.
> Recommended: Use `pool_block_size = 32` (default) for most navigation meshes.

> [!NOTE]
> Path caching provides 10-100× speedup for repeated paths (e.g., tower defense, RTS games).
> - Set `cache_size=0` to disable (zero overhead)
> - Recommended `cache_size=16-128` for typical games
> - Paths invalidated automatically on navmesh changes (via version tracking)

**Example:**
```lua
function init(self)
    -- Initialize navmesh with 600 cells, 6 edges per cell
    pathfinder.navmesh_init(
        600,   -- max_cells
        6,     -- max_edges_per_cell
        32,    -- pool_block_size
        16,    -- cache_size
        256,   -- max_cache_path_length
        5,     -- min_cell_size
        10,    -- max_cell_size
        1000,  -- max_grid_dim
        false  -- debug
    )
end
```

### pathfinder.navmesh_shutdown()

Shutdown and cleanup the navigation mesh system. Deallocates all memory and resets version counters.

**Syntax:**
```lua
pathfinder.navmesh_shutdown()
```

**Example:**
```lua
function final(self)
    pathfinder.navmesh_shutdown()
end
```

---

## Configuration

### pathfinder.navmesh_set_funnel()

Configure the funnel algorithm tolerances for path smoothing. Must be called AFTER `navmesh_init()` to customize funnel algorithm behavior.

**Syntax:**
```lua
pathfinder.navmesh_set_funnel(portal_vertex_tolerance, portal_collapse_threshold, waypoint_duplicate_tolerance)
```

**Parameters:**
- `portal_vertex_tolerance` (number): Tolerance for vertex matching in portal extraction (default: 0.002)
- `portal_collapse_threshold` (number): Threshold for collapsing narrow portals (default: 0.1)
- `waypoint_duplicate_tolerance` (number): Tolerance for duplicate waypoint filtering (default: 0.001)

**When to customize:**
- **Large world scales** (e.g., 1 unit = 1 kilometer): Increase tolerances
- **Small world scales** (e.g., 1 unit = 1 centimeter): Decrease tolerances
- **Portal extraction failures**: Increase `portal_vertex_tolerance`
- **Need higher precision**: Decrease `waypoint_duplicate_tolerance`

**Example:**
```lua
function init(self)
    -- Initialize navmesh first
    pathfinder.navmesh_init(600, 6, 32, 16, 256)
    
    -- Customize funnel tolerances for large world
    pathfinder.navmesh_set_funnel(
        0.005,  -- portal_vertex_tolerance (increased for large scale)
        0.2,    -- portal_collapse_threshold
        0.01    -- waypoint_duplicate_tolerance
    )
end
```

---

## Buffer Management

### pathfinder.navmesh_set_buffer()

Load navigation mesh data from a Defold buffer. The buffer must contain vertex positions defining the polygon cells of the navigation mesh.

**Syntax:**
```lua
pathfinder.navmesh_set_buffer(buffer)
```

**Parameters:**
- `buffer` (buffer): Defold buffer containing navigation mesh vertex data

**Buffer Format:**
The buffer must have a stream named `"position"` with 3 float32 values per vertex (x, y, z). Vertices are grouped into polygons, with each polygon's vertices stored consecutively.

**Example:**
```lua
-- Using a buffer resource
go.property("navmesh_buffer", resource.buffer("/assets/navmesh.buffer"))

function init(self)
    -- Initialize navmesh system
    pathfinder.navmesh_init(600, 6, 32, 16, 256)
    
    -- Load buffer from resource
    local buffer = resource.get_buffer(self.navmesh_buffer)
    pathfinder.navmesh_set_buffer(buffer)
    
    -- Navmesh is now ready for pathfinding
end
```

**Buffer Creation:**
Navigation mesh buffers are typically generated from 3D modeling tools or navmesh generation libraries (e.g., Recast Navigation). The buffer should contain:
- Stream name: `"position"`
- Value type: `buffer.VALUE_TYPE_FLOAT32`
- Components: 3 (x, y, z coordinates)

---

## Pathfinding

### pathfinder.navmesh_find_path()

Find a smoothed path through the navigation mesh from start to goal position using Polygon A* and Funnel algorithm.

**Syntax:**
```lua
local path_length, status, status_text, path = pathfinder.navmesh_find_path(start_x, start_y, goal_x, goal_y, max_path_length, agent_radius, enable_fallback)
```

**Parameters:**
- `start_x` (number): X coordinate of start position
- `start_y` (number): Y coordinate of start position (typically Z in 3D)
- `goal_x` (number): X coordinate of goal position
- `goal_y` (number): Y coordinate of goal position (typically Z in 3D)
- `max_path_length` (number): Maximum path length in waypoints
- `agent_radius` (number)[optional, default: 0.0]: Agent radius for collision avoidance (0 = no offset)
- `enable_fallback` (boolean)[optional, default: false]: Use nearest cell when position not in any cell

**Returns:**
- `path_length` (number): Number of waypoints in the path
- `status` (number): PathStatus code indicating success or error
- `status_text` (string): Human-readable status message
- `path` (PathNode[]): Array of waypoint positions with x and y coordinates

**Algorithm Pipeline:**
1. **Cell Lookup**: Find cells containing start and goal positions using spatial index
2. **Polygon A\***: Find corridor of adjacent cells from start cell to goal cell
3. **Portal Extraction**: Extract shared edges (portals) between consecutive cells
4. **Portal Offsetting**: If `agent_radius > 0`, offset portals inward for collision avoidance
5. **Funnel Algorithm**: Apply SSFA to find optimal shortest path through portals

**Status Codes:**
- `SUCCESS`: Path found, both positions in cells
- `SUCCESS_START_FALLBACK`: Path found, start position used nearest cell
- `SUCCESS_GOAL_FALLBACK`: Path found, goal position used nearest cell
- `ERROR_START_NOT_IN_CELL`: Start position not in any cell, fallback disabled
- `ERROR_GOAL_NOT_IN_CELL`: Goal position not in any cell, fallback disabled
- `ERROR_NO_PATH`: No cell corridor exists between positions
- `ERROR_HEAP_FULL`: Heap pool exhausted during A*

**Example:**
```lua
function on_input(self, action_id, action)
    if action_id == hash("mouse_click") and action.pressed then
        -- Convert screen to world position
        local world_pos = screen_to_world(action.x, action.y)
        
        -- Find path from player to clicked position
        local path_length, status, status_text, path = pathfinder.navmesh_find_path(
            self.player_pos.x,  -- start_x
            self.player_pos.z,  -- start_y (Z in 3D)
            world_pos.x,        -- goal_x
            world_pos.z,        -- goal_y
            128,                -- max_path_length
            0.5,                -- agent_radius (0.5 units for collision)
            true                -- enable_fallback
        )
        
        if status == pathfinder.PathStatus.SUCCESS then
            print("Path found with", path_length, "waypoints")
            self.current_path = path
            self.path_index = 1
        elseif status == pathfinder.PathStatus.SUCCESS_START_FALLBACK then
            print("Path found, but start was outside navmesh")
            -- Move player to first waypoint first
            self.current_path = path
            self.path_index = 1
        elseif status == pathfinder.PathStatus.ERROR_NO_PATH then
            print("No path exists to target")
        else
            print("Pathfinding failed:", status_text)
        end
    end
end

function update(self, dt)
    -- Follow path
    if self.current_path and self.path_index <= #self.current_path then
        local waypoint = self.current_path[self.path_index]
        local target = vmath.vector3(waypoint.x, 0, waypoint.y)
        
        -- Move towards waypoint
        local dir = vmath.normalize(target - self.player_pos)
        self.player_pos = self.player_pos + dir * self.speed * dt
        
        -- Check if reached waypoint
        if vmath.length(target - self.player_pos) < 0.5 then
            self.path_index = self.path_index + 1
        end
    end
end
```

**Agent Radius Behavior:**
- `agent_radius = 0`: No portal offsetting, path hugs walls
- `agent_radius > 0`: Portals offset inward by radius, provides collision clearance
- Narrow portals (< `agent_radius * collapse_threshold`) collapse to midpoint

### pathfinder.navmesh_cell_at_position()

Find which navigation mesh cell contains a given position.

**Syntax:**
```lua
local cell_id, center_x, center_y = pathfinder.navmesh_cell_at_position(x, y)
```

**Parameters:**
- `x` (number): X coordinate of position to query
- `y` (number): Y coordinate of position to query (typically Z in 3D)

**Returns:**
- `cell_id` (number): ID of cell containing position, or `pathfinder.INVALID_ID` (4294967295) if not found
- `center_x` (number): X coordinate of cell center
- `center_y` (number): Y coordinate of cell center

**Description:**

Uses the spatial grid index for O(1) average lookup, then performs point-in-polygon tests on candidate cells. Useful for:
- Determining if a position is walkable
- Getting cell information for debug visualization
- Validating spawn points or target positions

**Example:**
```lua
function validate_spawn_point(self, x, z)
    local cell_id, center_x, center_y = pathfinder.navmesh_cell_at_position(x, z)
    
    if cell_id ~= pathfinder.INVALID_ID then
        print("Position is walkable, in cell", cell_id)
        print("Cell center:", center_x, center_y)
        return true
    else
        print("Position is not walkable (outside navmesh)")
        return false
    end
end
```

---

## Spatial Index

### pathfinder.navmesh_get_spatial_index()

Get spatial index grid data for debug visualization. The spatial index is a grid-based structure that accelerates cell lookup during pathfinding.

**Syntax:**
```lua
local grid = pathfinder.navmesh_get_spatial_index()
```

**Returns:**
- `grid` (table): Table with `vertical` and `horizontal` arrays, each containing line data

**Grid Structure:**
- `vertical` (array): Array of vertical grid lines
- `horizontal` (array): Array of horizontal grid lines
- Each line contains:
  - `start_position` (vector3): Start point of grid line
  - `end_position` (vector3): End point of grid line

**Description:**

Returns the spatial index grid structure for rendering debug overlays. This visualizes how the navigation mesh is spatially partitioned for efficient queries. The grid cell size is automatically calculated from the polygon sizes and clamped to min/max values specified in `navmesh_init()`.

**Example:**
```lua
function init(self)
    pathfinder.navmesh_init(600, 6, 32, 16, 256)
    
    local buffer = resource.get_buffer(self.navmesh_buffer)
    pathfinder.navmesh_set_buffer(buffer)
    
    -- Get spatial index for debug rendering
    self.grid = pathfinder.navmesh_get_spatial_index()
end

function update(self, dt)
    -- Draw spatial index grid
    if self.grid.vertical then
        for _, line in ipairs(self.grid.vertical) do
            msg.post("@render:", "draw_line", {
                start_point = line.start_position,
                end_point = line.end_position,
                color = vmath.vector4(1, 0, 0, 0.3)  -- Red, semi-transparent
            })
        end
    end
    
    if self.grid.horizontal then
        for _, line in ipairs(self.grid.horizontal) do
            msg.post("@render:", "draw_line", {
                start_point = line.start_position,
                end_point = line.end_position,
                color = vmath.vector4(1, 0, 0, 0.3)  -- Red, semi-transparent
            })
        end
    end
end
```

---

## Statistics

### pathfinder.navmesh_get_stats()

Get comprehensive statistics about navmesh pathfinding caches for performance monitoring and optimization.

**Syntax:**
```lua
local stats = pathfinder.navmesh_get_stats()
```

**Returns:**
- `stats` (table): Table containing cache statistics

**Fields:**
- `path_cache` (table): Path cache statistics
  - `cache_entries` (number): Current number of cached paths
  - `cache_capacity` (number): Maximum cache capacity
  - `cache_hit_rate` (number): Cache hit rate percentage (0-100)
- `distance_cache` (table): Distance cache statistics
  - `current_size` (number): Current number of cached distances
  - `hit_count` (number): Number of cache hits
  - `miss_count` (number): Number of cache misses
  - `hit_rate` (number): Cache hit rate percentage (0-100)

**Description:**

Provides detailed performance metrics for monitoring navmesh pathfinding efficiency. Use this data to:
- Tune cache sizes for optimal performance
- Monitor cache effectiveness
- Identify performance bottlenecks
- Validate optimization strategies

**Example:**
```lua
function update(self, dt)
    -- Update stats every second
    self.stats_timer = (self.stats_timer or 0) + dt
    if self.stats_timer >= 1.0 then
        self.stats_timer = 0
        
        local stats = pathfinder.navmesh_get_stats()
        
        -- Log path cache stats
        print(string.format("Path Cache: %d/%d entries, Hit Rate: %d%%",
            stats.path_cache.cache_entries,
            stats.path_cache.cache_capacity,
            stats.path_cache.cache_hit_rate
        ))
        
        -- Log distance cache stats
        print(string.format("Distance Cache: %d entries, Hit Rate: %d%%",
            stats.distance_cache.current_size,
            stats.distance_cache.hit_rate
        ))
        
        -- Warn if cache efficiency is low
        if stats.path_cache.cache_hit_rate < 20 then
            print("WARNING: Low path cache hit rate, consider increasing cache_size")
        end
    end
end
```

---

## Enumerations

### PathStatus

Status codes for navmesh pathfinding operations.

| Constant | Value | Description |
|----------|-------|-------------|
| `SUCCESS` | 0 | Operation completed successfully, both positions in cells |
| `SUCCESS_START_FALLBACK` | 1 | Path found, but start position used fallback to nearest cell |
| `SUCCESS_GOAL_FALLBACK` | 2 | Path found, but goal position used fallback to nearest cell |
| `ERROR_NO_PATH` | -1 | No valid path found between start and goal positions |
| `ERROR_START_NODE_INVALID` | -2 | Invalid or unwalkable start cell |
| `ERROR_GOAL_NODE_INVALID` | -3 | Invalid or unwalkable goal cell |
| `ERROR_START_NOT_IN_CELL` | -13 | Start position not in any cell, fallback disabled |
| `ERROR_GOAL_NOT_IN_CELL` | -14 | Goal position not in any cell, fallback disabled |
| `ERROR_HEAP_FULL` | -6 | Heap pool exhausted during pathfinding (increase pool_block_size) |

**Fallback Behavior:**

When `enable_fallback = true` in `navmesh_find_path()`:
- Position not in any cell → Uses nearest cell by center distance
- Returns `SUCCESS_START_FALLBACK` or `SUCCESS_GOAL_FALLBACK` status
- First waypoint is the entry point to nearest cell

When `enable_fallback = false`:
- Position not in any cell → Returns error
- Returns `ERROR_START_NOT_IN_CELL` or `ERROR_GOAL_NOT_IN_CELL` status
- Useful for rejecting invalid clicks on walls/obstacles

**Usage:**
```lua
local path_length, status, status_text, path = pathfinder.navmesh_find_path(
    start_x, start_y, goal_x, goal_y, 128, 0.5, true
)

if status == pathfinder.PathStatus.SUCCESS then
    -- Normal path, both positions were in cells
    follow_path(path)
elseif status == pathfinder.PathStatus.SUCCESS_START_FALLBACK then
    -- Start was outside navmesh, moved to nearest cell
    -- First waypoint is the corrected start position
    move_to_position(path[1])
    follow_path(path)
elseif status == pathfinder.PathStatus.SUCCESS_GOAL_FALLBACK then
    -- Goal was outside navmesh, moved to nearest cell
    -- Agent will reach nearest valid position
    follow_path(path)
elseif status == pathfinder.PathStatus.ERROR_START_NOT_IN_CELL then
    -- Start position invalid and fallback disabled
    show_error("Cannot start from this position")
elseif status == pathfinder.PathStatus.ERROR_NO_PATH then
    -- No path exists between cells
    show_error("No path to target")
else
    print("Pathfinding failed:", status_text)
end
```

---

## Performance Tips

### Cache Configuration

**For Tower Defense / RTS:**
```lua
pathfinder.navmesh_init(
    600,   -- max_cells
    6,     -- max_edges_per_cell
    32,    -- pool_block_size
    128,   -- cache_size (high for repeated queries)
    256    -- max_cache_path_length
)
```

**For Action / Adventure:**
```lua
pathfinder.navmesh_init(
    600,   -- max_cells
    6,     -- max_edges_per_cell
    32,    -- pool_block_size
    16,    -- cache_size (lower for varied paths)
    256    -- max_cache_path_length
)
```

### Heap Pool Sizing

- **Small meshes** (<200 cells): `pool_block_size = 32`
- **Medium meshes** (200-500 cells): `pool_block_size = 64`
- **Large meshes** (>500 cells): `pool_block_size = 128`



### Fallback Strategy

- **Permissive**: `enable_fallback = true` - Always finds a path
- **Strict**: `enable_fallback = false` - Only paths between valid positions
- Check status codes to handle fallback cases appropriately

---
