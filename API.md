# Pathfinder API Documentation

Defold Graph Pathfinder Extension - High-performance A* pathfinding library for real-time games and simulations.

## Table of Contents

- [Initialization](#initialization)
- [Node Management](#node-management)
- [Edge Management](#edge-management)
- [Pathfinding](#pathfinding)
- [Path Smoothing](#path-smoothing)
- [Game Object Nodes](#game-object-nodes)
- [Cache Statistics](#cache-statistics)
- [Enumerations](#enumerations)
- [Data Types](#data-types)

---

## Initialization

### pathfinder.init()

Initialize the pathfinding system. Must be called before any other pathfinding operations.

**Syntax:**
```lua
pathfinder.init(max_nodes, [max_gameobject_nodes], max_edges_per_node, heap_pool_block_size, max_cache_path_length)
```

**Parameters:**
- `max_nodes` (number): Maximum number of nodes in the graph. Minimum value is 32. 
- `max_gameobject_nodes` (number|nil)[optional, default: 0]: Maximum number of game object nodes
- `max_edges_per_node` (number): Maximum edges per node
- `heap_pool_block_size` (number): Size of heap pool blocks for A* algorithm
- `max_cache_path_length` (number): Maximum length of cached paths

> [!IMPORTANT]
>The heap pool capacity equals `max_nodes`. If `heap_pool_block_size` > `max_nodes`, it will be automatically clamped to `max_nodes` to prevent heap allocation failures.
>Recommended: Use `heap_pool_block_size = 32` (default) and ensure `max_nodes >= 32`. 

> [!IMPORTANT]
> If you plan to use projected pathfinding, consider increasing the value of `max_nodes`. Each entry point is treated as a new node.  
> **Example:** If your graph has 100 nodes and 100 agents are constantly finding projected paths, you'll have up to 100 additional entry points — resulting in a total of 200 nodes.

[More info and QA about](./API.md#understanding-heap_pool_block_size-deep-dive) `heap_pool_block_size` and  `max_nodes`

**Example:**
```lua
-- Initialize with 100 nodes, 10 game object nodes, 4 edges per node
pathfinder.init(100, 10, 4, 32, 32)
```

### pathfinder.shutdown()

Shutdown the pathfinding system and free all resources.

**Syntax:**
```lua
pathfinder.shutdown()
```

**Example:**
```lua
function final(self)
    pathfinder.shutdown()
end
```

---

## Node Management

### pathfinder.add_node()

Add a single node to the pathfinding graph.

**Syntax:**
```lua
local node_id = pathfinder.add_node(x, y)
```

**Parameters:**
- `x` (number): X coordinate of the node
- `y` (number): Y coordinate of the node

**Returns:**
- `node_id` (number): Unique identifier for the created node

**Example:**
```lua
local node1 = pathfinder.add_node(100, 200)
local node2 = pathfinder.add_node(300, 400)
```

### pathfinder.add_nodes()

Add multiple nodes to the pathfinding graph in batch.

**Syntax:**
```lua
local node_ids = pathfinder.add_nodes(node_positions)
```

**Parameters:**
- `node_positions` (PathNode[]): Array of node positions with x and y coordinates

**Returns:**
- `node_ids` (number[]): Array of created node IDs

**Example:**
```lua
local nodes = pathfinder.add_nodes({
    { x = 100, y = 200 },
    { x = 200, y = 300 },
    { x = 300, y = 400 }
})
-- nodes = {0, 1, 2}
```

### pathfinder.remove_node()

Remove a node from the pathfinding graph.

**Syntax:**
```lua
pathfinder.remove_node(node_id)
```

**Parameters:**
- `node_id` (number): ID of the node to remove

**Example:**
```lua
pathfinder.remove_node(node_id)
```

### pathfinder.move_node()

Move an existing node to a new position.

**Syntax:**
```lua
pathfinder.move_node(node_id, x, y)
```

**Parameters:**
- `node_id` (number): ID of the node to move
- `x` (number): New X coordinate
- `y` (number): New Y coordinate

**Example:**
```lua
pathfinder.move_node(node1, 150, 250)
```

### pathfinder.get_node_position()

Get the position of a node.

**Syntax:**
```lua
local position = pathfinder.get_node_position(node_id)
```

**Parameters:**
- `node_id` (number): ID of the node

**Returns:**
- `position` (PathNode): Table with x and y coordinates

**Example:**
```lua
local pos = pathfinder.get_node_position(node1)
print("Node position:", pos.x, pos.y)
```

---

## Edge Management

### pathfinder.add_edge()

Add a single edge between two nodes.

**Syntax:**
```lua
pathfinder.add_edge(from_node_id, to_node_id, bidirectional, cost)
```

**Parameters:**
- `from_node_id` (number): Source node ID
- `to_node_id` (number): Target node ID
- `bidirectional` (boolean): If true, creates edges in both directions
- `cost` (number|nil): Optional edge cost (default: Euclidean distance between nodes)

**Example:**
```lua
-- Create a bidirectional edge with default cost
pathfinder.add_edge(node1, node2, true)

-- Create a one-way edge with custom cost
pathfinder.add_edge(node2, node3, false, 100)
```

### pathfinder.add_edges()

Add multiple edges to the pathfinding graph in batch.

**Syntax:**
```lua
pathfinder.add_edges(edges)
```

**Parameters:**
- `edges` (PathEdge[]): Array of edge definitions

**Example:**
```lua
pathfinder.add_edges({
    { from_node_id = node1, to_node_id = node2, bidirectional = true },
    { from_node_id = node2, to_node_id = node3, bidirectional = true, cost = 150 },
    { from_node_id = node2, to_node_id = node3 }
})
```

### pathfinder.remove_edge()

Remove an edge between two nodes.

**Syntax:**
```lua
pathfinder.remove_edge(from_node_id, to_node_id, [bidirectional])
```

**Parameters:**
- `from_node_id` (number): Source node ID
- `to_node_id` (number): Target node ID
- `bidirectional` (boolean)[optional, default: false]: If true, removes edges in both directions

**Example:**
```lua
pathfinder.remove_edge(node1, node2, true)
```

---

## Pathfinding

### pathfinder.find_path()

Find a path between two nodes using A* algorithm.

**Syntax:**
```lua
local path_length, status, status_text, path = pathfinder.find_path(start_node_id, goal_node_id, max_path_length, [smooth_id])
```

**Parameters:**
- `start_node_id` (number): Starting node ID
- `goal_node_id` (number): Goal node ID
- `max_path_length` (number): Maximum path length to search
- `smooth_id` (number|nil) [optional, default: 0 = no smoothing]: Optional smoothing configuration ID

**Returns:**
- `path_length` (number): Number of waypoints in the path
- `status` (number): PathStatus code indicating success or error
- `status_text` (string): Human-readable status message
- `path` (PathNode[]): Array of waypoints (positions with optional node IDs)

**Example:**
```lua
local path_length, status, status_text, path = pathfinder.find_path(start_id, goal_id, 128)

if status == pathfinder.PathStatus.SUCCESS then
    print("Path found with", path_length, "waypoints")
    for i, waypoint in ipairs(path) do
        print("Waypoint", i, ":", waypoint.x, waypoint.y, "Node ID:", waypoint.id)
    end
else
    print("Pathfinding failed:", status_text)
end
```

### pathfinder.find_projected_path()

Find a path from an arbitrary position (not on graph) to a goal node. Projects the start position onto the nearest graph edge and pathfinds from there.

**Syntax:**
```lua
local path_length, status, status_text, entry_point, path = pathfinder.find_projected_path(x, y, goal_node_id, max_path_length, [smooth_id])
```

**Parameters:**
- `x` (number): X coordinate of start position
- `y` (number): Y coordinate of start position
- `goal_node_id` (number): Goal node ID
- `max_path_length` (number): Maximum path length to search
- `smooth_id` (number|nil)[optionla, default: 0 = no smoothing]: Optional smoothing configuration ID 

**Returns:**
- `path_length` (number): Number of waypoints in the path
- `status` (number): PathStatus code indicating success or error
- `status_text` (string): Human-readable status message
- `entry_point` (vector3): Position where the path enters the graph
- `path` (PathNode[]): Array of waypoints (positions with optional node IDs)

**Example:**
```lua
local mouse_x, mouse_y = 150, 250
local path_length, status, status_text, entry_point, path = pathfinder.find_projected_path(mouse_x, mouse_y, goal_id, 128)

if status == pathfinder.PathStatus.SUCCESS then
    print("Entry point:", entry_point.x, entry_point.y)
    -- Draw line from mouse position to entry point
    -- Then follow the path
end
```

---

## Path Smoothing

### pathfinder.add_path_smoothing()

Create a path smoothing configuration.

**Syntax:**
```lua
local smooth_id = pathfinder.add_path_smoothing(config)
```

**Parameters:**
- `config` (PathSmoothConfig): Smoothing configuration table

**Returns:**
- `smooth_id` (number): Unique identifier for the smoothing configuration

**Example:**
```lua
local smooth_config = {
    style = pathfinder.PathSmoothStyle.BEZIER_QUADRATIC,
    bezier_sample_segment = 8,
    bezier_curve_radius = 0.8
}

local smooth_id = pathfinder.add_path_smoothing(smooth_config)

-- Use in pathfinding
local path_length, status, status_text, path = pathfinder.find_path(start_id, goal_id, 128, smooth_id)
```

### pathfinder.smooth_path()

Apply path smoothing to a set of waypoints.

**Syntax:**
```lua
local smoothed_length, smoothed_path = pathfinder.smooth_path(smooth_id, waypoints)
```

**Parameters:**
- `smooth_id` (number): Smoothing configuration ID (from add_path_smoothing)
- `waypoints` (PathNode[]): Array of waypoint positions

**Returns:**
- `smoothed_length` (number): Number of points in smoothed path
- `smoothed_path` (PathNode[]): Array of smoothed positions

**Example:**
```lua
local waypoints = {
    { x = 100, y = 100 },
    { x = 200, y = 200 },
    { x = 300, y = 150 }
}
local smoothed_length, smoothed_path = pathfinder.smooth_path(smooth_id, waypoints)
```

---

## Game Object Nodes

### pathfinder.add_gameobject_node()

Add a game object node that automatically tracks the game object's position.

**Syntax:**
```lua
local node_id = pathfinder.add_gameobject_node(game_object_instance, [use_world_position])
```

**Parameters:**
- `game_object_instance` (userdata): Game object instance
- `use_world_position` (boolean)[optional, default: false]: Whether to use world or local position

**Returns:**
- `node_id` (number): Unique identifier for the created node

**Example:**
```lua
local go_url = msg.url("/enemy")
local node_id = pathfinder.add_gameobject_node(go_url, true)
```

### pathfinder.add_gameobject_nodes()

Add multiple game object nodes that automatically track their game objects' positions.

**Syntax:**
```lua
local node_ids = pathfinder.add_gameobject_nodes(game_object_nodes)
```

**Parameters:**
- `game_object_nodes` (GameObjectNodeConfig[]): Array of game object node configurations

**Returns:**
- `node_ids` (number[]): Array of created node IDs

**Example:**
```lua
local node_gos = {
    { msg.url("/enemy1"), true },   -- use world position
    { msg.url("/enemy2"), false },  -- use local position
    { msg.url("/enemy3") }          -- default to local position (false)
}
local go_node_ids = pathfinder.add_gameobject_nodes(node_gos)
```

### pathfinder.convert_gameobject_node()

Convert an existing node to a game object node.

**Syntax:**
```lua
pathfinder.convert_gameobject_node(node_id, game_object_instance, use_world_position)
```

**Parameters:**
- `node_id` (number): Existing node ID to convert
- `game_object_instance` (userdata): Game object instance to track
- `use_world_position` (boolean): Whether to use world or local position

**Example:**
```lua
local go_url = msg.url("/player")
pathfinder.convert_gameobject_node(existing_node_id, go_url, true)
```

### pathfinder.remove_gameobject_node()

Remove a game object node from tracking and the pathfinding graph.

**Syntax:**
```lua
pathfinder.remove_gameobject_node(node_id)
```

**Parameters:**
- `node_id` (number): ID of the game object node to remove

**Example:**
```lua
pathfinder.remove_gameobject_node(node_id)
```

### pathfinder.pause_gameobject_node()

Pause automatic updates for a game object node.

**Syntax:**
```lua
pathfinder.pause_gameobject_node(node_id)
```

**Parameters:**
- `node_id` (number): ID of the game object node to pause

**Example:**
```lua
pathfinder.pause_gameobject_node(node_id)
```

### pathfinder.resume_gameobject_node()

Resume automatic updates for a game object node.

**Syntax:**
```lua
pathfinder.resume_gameobject_node(node_id)
```

**Parameters:**
- `node_id` (number): ID of the game object node to resume

**Example:**
```lua
pathfinder.resume_gameobject_node(node_id)
```

### pathfinder.gameobject_update()

Enable or disable automatic game object node position updates.

**Syntax:**
```lua
pathfinder.gameobject_update(enabled)
```

**Parameters:**
- `enabled` (boolean): True to enable automatic updates, false to disable

**Example:**
```lua
pathfinder.gameobject_update(true)  -- Enable automatic updates
```

### pathfinder.set_update_frequency()

Set the update frequency for game object node position updates. It is possible to set an independent update frequency for the game object position update iteration.
The default value is taken from the [display.frequency](https://defold.com/manuals/project-settings/#update-frequency) setting in the game.project file.
The update loop follows the same structure as in the [Defold source](https://github.com/defold/defold/blob/cdaa870389ca00062bfc03bcda8f4fb34e93124a/engine/engine/src/engine.cpp#L1860). 

**Syntax:**
```lua
pathfinder.set_update_frequency(frequency)
```

**Parameters:**
- `frequency` (number): Update frequency in Hz

**Example:**
```lua
pathfinder.set_update_frequency(60)  -- Update at 60 Hz
```

---

## Cache Statistics

### pathfinder.get_cache_stats()

Get caching statistics for pathfinding operations. Returns detailed statistics about path caching and distance caching performance.

**Syntax:**
```lua
local stats = pathfinder.get_cache_stats()
```

**Returns:**
- `stats` (CacheStats): Cache statistics with path_cache and distance_cache fields

**Example:**
```lua
local stats = pathfinder.get_cache_stats()

print("Path Cache:")
print("  Current Entries:", stats.path_cache.current_entries)
print("  Max Capacity:", stats.path_cache.max_capacity)
print("  Hit Rate:", stats.path_cache.hit_rate .. "%")

print("Distance Cache:")
print("  Current Size:", stats.distance_cache.current_size)
print("  Hit Count:", stats.distance_cache.hit_count)
print("  Miss Count:", stats.distance_cache.miss_count)
print("  Hit Rate:", stats.distance_cache.hit_rate .. "%")
```

---

## Enumerations

### PathStatus

Status codes for pathfinding operations.

| Constant | Value | Description |
|----------|-------|-------------|
| `SUCCESS` | 0 | Operation completed successfully |
| `ERROR_NO_PATH` | -1 | No valid path found between start and goal nodes |
| `ERROR_START_NODE_INVALID` | -2 | Invalid or inactive start node ID |
| `ERROR_GOAL_NODE_INVALID` | -3 | Invalid or inactive goal node ID |
| `ERROR_NODE_FULL` | -4 | Node capacity reached, cannot add more nodes |
| `ERROR_EDGE_FULL` | -5 | Edge capacity reached, cannot add more edges |
| `ERROR_HEAP_FULL` | -6 | Heap pool exhausted during pathfinding |
| `ERROR_PATH_TOO_LONG` | -7 | Path exceeds maximum allowed length |
| `ERROR_GRAPH_CHANGED` | -8 | Graph modified during pathfinding (retrying) |
| `ERROR_GRAPH_CHANGED_TOO_OFTEN` | -11 | Graph changed too often during pathfinding |
| `ERROR_NO_PROJECTION` | -9 | Cannot project point onto graph (no edges exist) |
| `ERROR_VIRTUAL_NODE_FAILED` | -10 | Failed to create or connect virtual node |

**Usage:**
```lua
if status == pathfinder.PathStatus.SUCCESS then
    -- Path found successfully
elseif status == pathfinder.PathStatus.ERROR_NO_PATH then
    -- No path exists
end
```

### PathSmoothStyle

Path smoothing algorithms.

| Constant | Value | Description | Best For |
|----------|-------|-------------|----------|
| `NONE` | 0 | No smoothing (angular paths, fastest) | Grid-based movement |
| `CATMULL_ROM` | 1 | Passes through all waypoints with smooth curves | Precise waypoint following |
| `BEZIER_CUBIC` | 2 | Very smooth curves with two control points | Cinematic cameras, UI |
| `BEZIER_QUADRATIC` | 3 | Corner-only smoothing (recommended) | Character movement, vehicles |
| `BEZIER_ADAPTIVE` | 4 | Adaptive corner smoothing (highly customizable) | Dynamic paths, varying turn angles |
| `CIRCULAR_ARC` | 5 | Perfect circular arcs (best for tile-based games) | Railroads, tower defense |

 `BEZIER_QUADRATIC` offers good balance between quality and performance.

**Usage:**
```lua
local config = {
    style = pathfinder.PathSmoothStyle.BEZIER_QUADRATIC,
    bezier_sample_segment = 8,
    bezier_curve_radius = 0.8
}
```

---

## Data Types

### PathNode

Represents a node position in the pathfinding graph.

**Fields:**
- `x` (number): X coordinate of the node position
- `y` (number): Y coordinate of the node position


### PathEdge

Represents an edge between two nodes.

**Fields:**
- `from_node_id` (number): Source node ID
- `to_node_id` (number): Target node ID
- `bidirectional` (boolean)[optional]: Whether the edge is bidirectional
- `cost` (number|nil)[optional]: Optional edge cost (default: Euclidean distance)

### PathSmoothConfig

Configuration for path smoothing.

**Fields:**
- `style` (number): Path smoothing style (use pathfinder.PathSmoothStyle constants)
- `bezier_sample_segment` (number): Number of samples per segment for Bezier curves (default: 8)
- `bezier_control_point_offset` (number)[0.0-1.0, default: 0.4]: Control point offset for **BEZIER_CUBIC** style 
- `bezier_curve_radius` (number) [0.0-1.0, default: 0.8]: Curve radius for **BEZIER_QUADRATIC** style
- `bezier_adaptive_tightness` (number)[default: 0.5]: Tightness for **BEZIER_ADAPTIVE** style 
- `bezier_adaptive_roundness` (number)[default: 0.5]: Roundness for **BEZIER_ADAPTIVE** style 
- `bezier_adaptive_max_corner_distance` (number][default: 50.0]: Maximum corner distance for **BEZIER_ADAPTIVE** 
- `bezier_arc_radius` (number)[default: 60.0]: Arc radius for **CIRCULAR_ARC** style 



### GameObjectNodeConfig

Configuration for a game object node (used in add_gameobject_nodes).

**Fields:**
- `[1]` (msg.url): Game object instance
- `[2]` (boolean|nil)[optional, default: false if omitted]:  Whether to use world position 

### CacheStats

Cache statistics returned by get_cache_stats().

**Fields:**
- `path_cache` (PathCacheStats): Path cache statistics
- `distance_cache` (DistanceCacheStats): Distance cache statistics

### PathCacheStats

Path cache statistics.

**Fields:**
- `current_entries` (number): Current number of cached paths
- `max_capacity` (number): Maximum cache capacity
- `hit_rate` (number): Cache hit rate percentage (0-100)

### DistanceCacheStats

Distance cache statistics.

**Fields:**
- `current_size` (number): Current number of cached distance calculations
- `hit_count` (number): Number of cache hits
- `miss_count` (number): Number of cache misses
- `hit_rate` (number): Cache hit rate percentage (0-100)

---

## Complete Example

```lua
function init(self)
    -- Initialize pathfinder
    pathfinder.init(100, 10, 4, 32, 32)
    
    -- Create nodes
    local nodes = pathfinder.add_nodes({
        { x = 100, y = 100 },
        { x = 200, y = 200 },
        { x = 300, y = 200 },
        { x = 400, y = 100 }
    })
    
    -- Create edges
    pathfinder.add_edges({
        { from_node_id = nodes[1], to_node_id = nodes[2], bidirectional = true },
        { from_node_id = nodes[2], to_node_id = nodes[3], bidirectional = true },
        { from_node_id = nodes[3], to_node_id = nodes[4], bidirectional = true }
    })
    
    -- Create smoothing configuration
    local smooth_config = {
        style = pathfinder.PathSmoothStyle.BEZIER_QUADRATIC,
        bezier_sample_segment = 8,
        bezier_curve_radius = 0.8
    }
    local smooth_id = pathfinder.add_path_smoothing(smooth_config)
    
    -- Find path with smoothing
    local start_id = nodes[1]
    local goal_id = nodes[4]
    local path_length, status, status_text, path = pathfinder.find_path(start_id, goal_id, 128, smooth_id)
    
    if status == pathfinder.PathStatus.SUCCESS then
        print("Path found with", path_length, "waypoints")
        for i, waypoint in ipairs(path) do
            print(string.format("Waypoint %d: (%.1f, %.1f)", i, waypoint.x, waypoint.y))
        end
    else
        print("Pathfinding failed:", status_text)
    end
    
    -- Get cache statistics
    local stats = pathfinder.get_cache_stats()
    print("Path cache hit rate:", stats.path_cache.hit_rate .. "%")
    print("Distance cache hit rate:", stats.distance_cache.hit_rate .. "%")
end

function final(self)
    pathfinder.shutdown()
end
```

---

## Understanding `heap_pool_block_size` (Deep Dive)

### What `heap_pool_block_size` Actually Represents

The `heap_pool_block_size` parameter defines **the maximum size of the A* algorithm's open set (priority queue)** during a single pathfinding operation.

#### A* Algorithm Open Set

During pathfinding, A* maintains an "open set" of nodes to explore:

1. **Start**: Open set contains only the start node
2. **Expansion**: Pop lowest-cost node, add its unexplored neighbors to open set
3. **Growth**: Open set grows as the search expands outward through the graph
4. **Peak**: Maximum size reached when search is widest (before finding goal)
5. **Goal Found**: Open set collapses as path is reconstructed

**The open set size depends on:**
- Graph density (more edges = larger open set)
- Path length (longer paths = wider search)
- Graph topology (grid vs. random graph)
- Heuristic quality (better heuristic = smaller open set)

### How to Determine Proper `heap_pool_block_size`

#### Method 1: Use Default (Recommended)

For 90% of use cases, **pool_block_size = 32** is sufficient:

```cpp
pathfinder::path::init(
    graph_nodes,  // Your graph size (e.g., 150)
    max_edges,    // Your max edges (e.g., 10)
    32,           // Default - works for most graphs
    8             // Cache length
);
```

**Works well for:**
- Small to medium graphs (< 500 nodes)
- Sparse graphs (average degree < 6)
- Short to medium paths (< 20 hops)
``

#### Method 2: Calculate Based on Graph Properties

For dense graphs or long paths, estimate based on graph characteristics:

```
// Formula: heap_pool_block_size ≈ sqrt(nodes_likely_to_explore)

// Sparse graph (grid, few connections):
heap_pool_block_size = 32;  // Open set rarely exceeds 32 nodes

// Medium density (navigation graph):
heap_pool_block_size = sqrt(total_nodes) / 2;  // e.g., sqrt(1000) / 2 ≈ 16

// Dense graph (many connections):
heap_pool_block_size = sqrt(total_nodes);  // e.g., sqrt(1000) ≈ 32

// Worst case (might explore entire graph):
heap_pool_block_size = total_nodes;  // Maximum possible
```


### Why pool_block_size Cannot Exceed max_nodes

#### The Memory Pool Architecture

The library uses a **single-buffer pooling strategy** for performance:

**At Init Time:**
```cpp
// Allocate ONE contiguous buffer
heap_pool_buffer = malloc(max_nodes * sizeof(HeapNode));
heap_pool_capacity = max_nodes;  // Total capacity
```

**During find_path():**
```cpp
// Each pathfinding operation requests a slice
heap_slice_start = heap_pool_buffer[current_offset];
heap_slice_size = pool_block_size;

// Check if slice fits in pool
if (current_offset + pool_block_size > heap_pool_capacity) {
    return ERROR_HEAP_FULL;  // Not enough space!
}

current_offset += pool_block_size;  // Reserve this slice
```

**The Constraint:**
- Pool capacity = `max_nodes`
- Request size = `pool_block_size`
- **If `pool_block_size > max_nodes`**: The first allocation fails because you're requesting more than the entire pool

**Why This Design?**

1. **Performance**: Single allocation at init time (no runtime malloc)
2. **Predictability**: Memory usage known upfront
3. **Cache-friendly**: Contiguous buffer, good memory locality
4. **Zero-copy**: Slices are just pointers into the buffer

**Trade-off**: `max_nodes` must be large enough to accommodate `pool_block_size`

#### Why It SHOULDN'T Work (By Design)

This behavior is **intentional**:

1. **Memory Safety**: Prevents buffer overruns
2. **Resource Limits**: Enforces declared capacity limits
3. **Predictability**: Fails fast rather than corrupting memory


### Real-World Examples

#### Example 1: Small Grid (20×20 = 400 nodes)
```lua
pathfinder.init(
    400,    // Graph has 400 nodes
    4,      // Grid: each node has max 4 neighbors
    32,     // Default: sufficient for grid searches
    8       // Typical path length on grid
);
```

#### Example 2: Navigation Mesh (150 nodes, dense)
```lua
pathfinder.init(
    150,    // Graph has 150 nodes
    10,     // Dense: up to 10 connections per node
    32,     // Default: works for most nav mesh queries
    10      // Nav mesh paths can be longer
);
```

#### Example 3: Large World Graph (5000 nodes)
```lua
pathfinder.init(
    5000,   // Graph has 5000 nodes
    8,      // Moderate connectivity
    64,     // Larger: anticipating complex searches
    12      // Longer paths in large world
);
```

---
