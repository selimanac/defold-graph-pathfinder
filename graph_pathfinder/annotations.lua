---@diagnostic disable: missing-return, unused-local

---@meta pathfinder
---Defold Graph Pathfinder Extension
---High-performance A* pathfinding library for real-time games and simulations.
---@class pathfinder

---@class PathNode
---@field x number X coordinate of the node position
---@field y number Y coordinate of the node position
---@field id number Node ID (only present in path results from find_path)

---@class PathEdge
---@field from_node_id number Source node ID
---@field to_node_id number Target node ID
---@field bidirectional boolean Whether the edge is bidirectional

---@class PathSmoothConfig
---@field style number Path smoothing style (use pathfinder.PathSmoothStyle constants)
---@field bezier_sample_segment number Number of samples per segment for Bezier curves (default: 8)
---@field bezier_control_point_offset number Control point offset for BEZIER_CUBIC style (0.0-1.0, default: 0.4)
---@field bezier_curve_radius number Curve radius for BEZIER_QUADRATIC style (0.0-1.0, default: 0.8)
---@field bezier_adaptive_tightness number Tightness for BEZIER_ADAPTIVE style (default: 0.5)
---@field bezier_adaptive_roundness number Roundness for BEZIER_ADAPTIVE style (default: 0.5)
---@field bezier_adaptive_max_corner_distance number Maximum corner distance for BEZIER_ADAPTIVE (default: 50.0)
---@field bezier_arc_radius number Arc radius for CIRCULAR_ARC style (default: 60.0)

---PathStatus enum - Status codes for pathfinding operations
---@enum PathStatus
pathfinder.PathStatus = {
    SUCCESS = 0,                         -- Operation completed successfully
    ERROR_NO_PATH = -1,                  -- No valid path found between start and goal nodes
    ERROR_START_NODE_INVALID = -2,       -- Invalid or inactive start node ID
    ERROR_GOAL_NODE_INVALID = -3,        -- Invalid or inactive goal node ID
    ERROR_NODE_FULL = -4,                -- Node capacity reached, cannot add more nodes
    ERROR_EDGE_FULL = -5,                -- Edge capacity reached, cannot add more edges
    ERROR_HEAP_FULL = -6,                -- Heap pool exhausted during pathfinding
    ERROR_PATH_TOO_LONG = -7,            -- Path exceeds maximum allowed length
    ERROR_GRAPH_CHANGED = -8,            -- Graph modified during pathfinding (retrying)
    ERROR_GRAPH_CHANGED_TOO_OFTEN = -11, -- Graph changed too often during pathfinding
    ERROR_NO_PROJECTION = -9,            -- Cannot project point onto graph (no edges exist)
    ERROR_VIRTUAL_NODE_FAILED = -10      -- Failed to create or connect virtual node
}

---PathSmoothStyle enum - Path smoothing algorithms
---@enum PathSmoothStyle
pathfinder.PathSmoothStyle = {
    NONE = 0,             -- No smoothing (angular paths, fastest)
    CATMULL_ROM = 1,      -- Passes through all waypoints with smooth curves
    BEZIER_CUBIC = 2,     -- Very smooth curves with two control points
    BEZIER_QUADRATIC = 3, -- Corner-only smoothing (recommended)
    BEZIER_ADAPTIVE = 4,  -- Adaptive corner smoothing (highly customizable)
    CIRCULAR_ARC = 5      -- Perfect circular arcs (best for tile-based games)
}

---Initialize the pathfinding system. Must be called before any other pathfinding operations.
---@param max_nodes number Maximum number of nodes in the graph
---@param max_gameobject_nodes number|nil Maximum number of game object nodes (optional, default: 0)
---@param max_edges_per_node number Maximum edges per node
---@param heap_pool_block_size number Size of heap pool blocks for A* algorithm
---@param max_cache_path_length number Maximum length of cached paths
function pathfinder.init(max_nodes, max_gameobject_nodes, max_edges_per_node, heap_pool_block_size, max_cache_path_length) end

---Shutdown the pathfinding system and free all resources.
function pathfinder.shutdown() end

---Add a single node to the pathfinding graph.
---@param x number X coordinate of the node
---@param y number Y coordinate of the node
---@return number node_id Unique identifier for the created node
function pathfinder.add_node(x, y) end

---Add multiple nodes to the pathfinding graph in batch.
---@param node_positions PathNode[] Array of node positions with x and y coordinates
---@return number[] node_ids Array of created node IDs
function pathfinder.add_nodes(node_positions) end

---Remove a node from the pathfinding graph.
---@param node_id number ID of the node to remove
function pathfinder.remove_node(node_id) end

---Move an existing node to a new position.
---@param node_id number ID of the node to move
---@param x number New X coordinate
---@param y number New Y coordinate
function pathfinder.move_node(node_id, x, y) end

---Get the position of a node.
---@param node_id number ID of the node
---@return PathNode position Table with x and y coordinates
function pathfinder.get_node_position(node_id) end

---Add a single edge between two nodes.
---@param from_node_id number Source node ID
---@param to_node_id number Target node ID
---@param bidirectional boolean If true, creates edges in both directions
function pathfinder.add_edge(from_node_id, to_node_id, bidirectional) end

---Add multiple edges to the pathfinding graph in batch.
---@param edges PathEdge[] Array of edge definitions
function pathfinder.add_edges(edges) end

---Remove an edge between two nodes.
---@param from_node_id number Source node ID
---@param to_node_id number Target node ID
---@param bidirectional boolean If true, removes edges in both directions
function pathfinder.remove_edge(from_node_id, to_node_id, bidirectional) end

---Find a path between two nodes using A* algorithm.
---@param start_node_id number Starting node ID
---@param goal_node_id number Goal node ID
---@param max_path_length number Maximum path length to search
---@param smooth_id number|nil Optional smoothing configuration ID (default: 0 = no smoothing)
---@return number path_length Number of waypoints in the path
---@return number status PathStatus code indicating success or error
---@return string status_text Human-readable status message
---@return PathNode[] path Array of waypoints (positions with optional node IDs)
function pathfinder.find_path(start_node_id, goal_node_id, max_path_length, smooth_id) end

---Find a path from an arbitrary position (not on graph) to a goal node.
---Projects the start position onto the nearest graph edge and pathfinds from there.
---@param x number X coordinate of start position
---@param y number Y coordinate of start position
---@param goal_node_id number Goal node ID
---@param max_path_length number Maximum path length to search
---@param smooth_id number|nil Optional smoothing configuration ID (default: 0 = no smoothing)
---@return number path_length Number of waypoints in the path
---@return number status PathStatus code indicating success or error
---@return string status_text Human-readable status message
---@return vector3 entry_point Position where the path enters the graph
---@return PathNode[] path Array of waypoints (positions with optional node IDs)
function pathfinder.find_projected_path(x, y, goal_node_id, max_path_length, smooth_id) end

---Apply path smoothing to a set of waypoints.
---@param smooth_id number Smoothing configuration ID (from add_path_smoothing)
---@param waypoints PathNode[] Array of waypoint positions
---@return number smoothed_length Number of points in smoothed path
---@return PathNode[] smoothed_path Array of smoothed positions
function pathfinder.smooth_path(smooth_id, waypoints) end

---Create a path smoothing configuration.
---@param config PathSmoothConfig Smoothing configuration table
---@return number smooth_id Unique identifier for the smoothing configuration
function pathfinder.add_path_smoothing(config) end

---Add a game object node that automatically tracks the game object's position.
---@param game_object_instance userdata Game object instance
---@param use_world_position boolean Whether to use world or local position
---@return number node_id Unique identifier for the created node
function pathfinder.add_gameobject_node(game_object_instance, use_world_position) end

---@class GameObjectNodeConfig
---@field [1] userdata Game object instance (msg.url)
---@field [2] boolean|nil Optional: Whether to use world position (default: false if omitted)

---Add multiple game object nodes that automatically track their game objects' positions.
---@param game_object_nodes GameObjectNodeConfig[] Array of game object node configurations
---@return number[] node_ids Array of created node IDs
function pathfinder.add_gameobject_nodes(game_object_nodes) end

---Convert an existing node to a game object node.
---@param node_id number Existing node ID to convert
---@param game_object_instance userdata Game object instance to track
---@param use_world_position boolean Whether to use world or local position
function pathfinder.convert_gameobject_node(node_id, game_object_instance, use_world_position) end

---Remove a game object node from tracking and the pathfinding graph.
---@param node_id number ID of the game object node to remove
function pathfinder.remove_gameobject_node(node_id) end

---Pause automatic updates for a game object node.
---@param node_id number ID of the game object node to pause
function pathfinder.pause_gameobject_node(node_id) end

---Resume automatic updates for a game object node.
---@param node_id number ID of the game object node to resume
function pathfinder.resume_gameobject_node(node_id) end

---Enable or disable automatic game object node position updates.
---@param enabled boolean True to enable automatic updates, false to disable
function pathfinder.gameobject_update(enabled) end

---Set the update frequency for game object node position updates.
---@param frequency number Update frequency in Hz
function pathfinder.set_update_frequency(frequency) end

---@class PathCacheStats
---@field current_entries number Current number of cached paths
---@field max_capacity number Maximum cache capacity
---@field hit_rate number Cache hit rate percentage (0-100)

---@class DistanceCacheStats
---@field current_size number Current number of cached distance calculations
---@field hit_count number Number of cache hits
---@field miss_count number Number of cache misses
---@field hit_rate number Cache hit rate percentage (0-100)

---@class CacheStats
---@field path_cache PathCacheStats Path cache statistics
---@field distance_cache DistanceCacheStats Distance cache statistics

---Get caching statistics for pathfinding operations.
---Returns detailed statistics about path caching and distance caching performance.
---@return CacheStats stats Cache statistics with path_cache and distance_cache fields
function pathfinder.get_cache_stats() end

return pathfinder
