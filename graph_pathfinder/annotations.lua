---@diagnostic disable: missing-return, unused-local

---@meta pathfinder
---Defold Graph Pathfinder Extension
---High-performance A* pathfinding library for real-time games and simulations.
---@class pathfinder

---@class PathNode
---@field x number X coordinate of the node position
---@field y number Y coordinate of the node position

---@class PathEdge
---@field from_node_id number Source node ID
---@field to_node_id number Target node ID
---@field bidirectional? boolean Optional: Whether the edge is bidirectional
---@field cost? number Optional: Edge cost (default: Euclidean distance between nodes)

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
    SUCCESS_START_FALLBACK = 1,          -- Success, but start position used fallback to nearest cell (navmesh only)
    SUCCESS_GOAL_FALLBACK = 2,           -- Success, but goal position used fallback to nearest cell (navmesh only)
    ERROR_NO_PATH = -1,                  -- No valid path found between start and goal nodes
    ERROR_START_GOAL_NODE_SAME = -12,    -- Start node ID and goal node ID are the same
    ERROR_START_NODE_INVALID = -2,       -- Invalid or inactive start node ID
    ERROR_GOAL_NODE_INVALID = -3,        -- Invalid or inactive goal node ID
    ERROR_START_NOT_IN_CELL = -13,       -- Start position not in any cell, fallback disabled (navmesh only)
    ERROR_GOAL_NOT_IN_CELL = -14,        -- Goal position not in any cell, fallback disabled (navmesh only)
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

---Get the edges of a node.
---@param node_id number ID of the node
---@param bidirectional? boolean If true, returns all edges. If false, returns only unidirectional edges. (default: true)
---@param include_incoming? boolean If true, includes incoming edges. If false, includes only outgoing edges.(default: false)
---@return PathEdge[]  Array of edge definitions
function pathfinder.get_node_edges(node_id, bidirectional, include_incoming) end

---Add a single edge between two nodes.
---@param from_node_id number Source node ID
---@param to_node_id number Target node ID
---@param bidirectional? boolean If true, creates edges in both directions
---@param cost? number Optional edge cost (default: Euclidean distance between nodes)
function pathfinder.add_edge(from_node_id, to_node_id, bidirectional, cost) end

---Add multiple edges to the pathfinding graph in batch.
---@param edges PathEdge[] Array of edge definitions
function pathfinder.add_edges(edges) end

---Remove an edge between two nodes.
---@param from_node_id number Source node ID
---@param to_node_id number Target node ID
---@param bidirectional? boolean If true, removes edges in both directions (default: false)
function pathfinder.remove_edge(from_node_id, to_node_id, bidirectional) end

---Find a path between two nodes using A* algorithm.
---@param start_node_id number Starting node ID
---@param goal_node_id number Goal node ID
---@param max_path_length number Maximum path length to search
---@param smooth_id? number|nil Optional smoothing configuration ID (default: 0 = no smoothing)
---@return number path_length Number of waypoints in the path
---@return number status PathStatus code indicating success or error
---@return string status_text Human-readable status message
---@return PathNode[] path Array of waypoints (positions with optional node IDs)
function pathfinder.find_node_to_node(start_node_id, goal_node_id, max_path_length, smooth_id) end

---Find a path from an arbitrary position (not on graph) to a goal node.
---Projects the start position onto the nearest graph edge and pathfinds from there.
---@param x number X coordinate of start position
---@param y number Y coordinate of start position
---@param goal_node_id number Goal node ID
---@param max_path_length number Maximum path length to search
---@param smooth_id? number|nil Optional smoothing configuration ID (default: 0 = no smoothing)
---@return number path_length Number of waypoints in the path
---@return number status PathStatus code indicating success or error
---@return string status_text Human-readable status message
---@return vector3 entry_point Position where the path enters the graph
---@return PathNode[] path Array of waypoints (positions with optional node IDs)
function pathfinder.find_projected_to_node(x, y, goal_node_id, max_path_length, smooth_id) end

---Find a path from a start node to an arbitrary position (not on graph).
---Projects the target position onto the nearest graph edge and pathfinds to there.
---@param start_node_id number Starting node ID
---@param x number X coordinate of target position
---@param y number Y coordinate of target position
---@param max_path_length number Maximum path length to search
---@param smooth_id? number|nil Optional smoothing configuration ID (default: 0 = no smoothing)
---@return number path_length Number of waypoints in the path
---@return number status PathStatus code indicating success or error
---@return string status_text Human-readable status message
---@return vector3 exit_point Position where the path exits the graph
---@return PathNode[] path Array of waypoints (positions with optional node IDs)
function pathfinder.find_node_to_projected(start_node_id, x, y, max_path_length, smooth_id) end

---Find a path from an arbitrary position (not on the graph) to another arbitrary position (not on the graph).
---Projects both start and target positions onto the nearest graph edges and pathfinds between them.
---@param start_x number X coordinate of start position
---@param start_y number Y coordinate of start position
---@param target_x number X coordinate of target position
---@param target_y number Y coordinate of target position
---@param max_path_length number Maximum path length to search
---@param smooth_id? number|nil Optional smoothing configuration ID (default: 0 = no smoothing)
---@return number path_length Number of waypoints in the path
---@return number status PathStatus code indicating success or error
---@return string status_text Human-readable status message
---@return vector3 entry_point Position where the path enters the graph
---@return vector3 exit_point Position where the path exits the graph
---@return PathNode[] path Array of waypoints (positions with optional node IDs)
function pathfinder.find_projected_to_projected(start_x, start_y, target_x, target_y, max_path_length, smooth_id) end

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

---Update a path smoothing configuration.
---@param smooth_id number Smoothing configuration ID (from add_path_smoothing)
---@param config PathSmoothConfig Smoothing configuration table
function pathfinder.update_path_smoothing(smooth_id, config) end

---Add a game object node that automatically tracks the game object's position.
---@param game_object_instance userdata Game object instance
---@param use_world_position? boolean Whether to use world or local position
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
---@param use_world_position? boolean Whether to use world or local position
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

---Get comprehensive statistics about pathfinding caches and spatial index.
---@return table stats Table containing cache and spatial index statistics with fields: path_cache, distance_cache, spatial_index
function pathfinder.get_stats() end

---Initialize the spatial index with custom configuration for accelerating projected pathfinding queries.
---@param max_grid_size number Maximum grid dimension (recommended: 1000)
---@param min_cell_size number Minimum cell size (recommended: 10.0)
---@param max_cell_size number Maximum cell size (recommended: 500.0)
---@param max_cell_search_radius number Search radius in cells (1 = 3×3 grid, 2 = 5×5 grid)
function pathfinder.set_spatial_index(max_grid_size, min_cell_size, max_cell_size, max_cell_search_radius) end

---Get spatial index grid data for debug visualization.
---@return table grid Table with vertical and horizontal arrays, each containing line data with start_position and end_position (vector3)
function pathfinder.get_spatial_index() end

---Check if the spatial index has been initialized and built.
---@return boolean is_initialized True if spatial index is active, false otherwise
function pathfinder.spatial_index_initialized() end

---Initialize the navigation mesh pathfinding system. Must be called before any other navmesh operations.
---@param max_cells number Maximum number of polygon cells in the navigation mesh
---@param max_edges_per_cell number Maximum edges/neighbors per cell (typically 3-8)
---@param pool_block_size number Heap pool block size for A* algorithm (default: 32)
---@param cache_size number Number of paths to cache (0 to disable, recommended: 16-128)
---@param max_cache_path_length number Maximum length of cached paths in cells (default: 256)
---@param min_cell_size? number Minimum spatial index grid cell size (default: 5.0)
---@param max_cell_size? number Maximum spatial index grid cell size (default: 10.0)
---@param max_grid_dim? number Maximum spatial index grid dimension (default: 1000)
---@param debug? boolean Enable debug output (default: false, requires NAVMESH_DEBUG=1 at compile time)
function pathfinder.navmesh_init(max_cells, max_edges_per_cell, pool_block_size, cache_size, max_cache_path_length, min_cell_size, max_cell_size, max_grid_dim, debug) end

---Shutdown and cleanup the navigation mesh system.
function pathfinder.navmesh_shutdown() end

---Configure the funnel algorithm tolerances for path smoothing. Must be called AFTER navmesh_init().
---@param portal_vertex_tolerance number Tolerance for vertex matching in portal extraction (default: 0.002)
---@param portal_collapse_threshold number Threshold for collapsing narrow portals (default: 0.1)
---@param waypoint_duplicate_tolerance number Tolerance for duplicate waypoint filtering (default: 0.001)
function pathfinder.navmesh_set_funnel(portal_vertex_tolerance, portal_collapse_threshold, waypoint_duplicate_tolerance) end

---Load navigation mesh data from a Defold buffer.
---@param buffer buffer Defold buffer containing navigation mesh vertex data
function pathfinder.navmesh_set_buffer(buffer) end

---Find a smoothed path through the navigation mesh using Polygon A* and Funnel algorithm.
---@param start_x number X coordinate of start position
---@param start_y number Y coordinate of start position (typically Z in 3D)
---@param goal_x number X coordinate of goal position
---@param goal_y number Y coordinate of goal position (typically Z in 3D)
---@param max_path_length number Maximum path length in waypoints
---@param agent_radius? number Agent radius for collision avoidance (default: 0.0, 0 = no offset)
---@param enable_fallback? boolean Use nearest cell when position not in any cell (default: false)
---@return number path_length Number of waypoints in the path
---@return number status PathStatus code indicating success or error
---@return string status_text Human-readable status message
---@return PathNode[] path Array of waypoint positions with x and y coordinates
function pathfinder.navmesh_find_path(start_x, start_y, goal_x, goal_y, max_path_length, agent_radius, enable_fallback) end

---Find which navigation mesh cell contains a given position.
---@param x number X coordinate of position to query
---@param y number Y coordinate of position to query (typically Z in 3D)
---@return number cell_id ID of cell containing position, or special value if not found
---@return number center_x X coordinate of cell center
---@return number center_y Y coordinate of cell center
function pathfinder.navmesh_cell_at_position(x, y) end

---Get spatial index grid data for debug visualization (navmesh).
---@return table grid Table with vertical and horizontal arrays, each containing line data with start_position and end_position (vector3)
function pathfinder.navmesh_get_spatial_index() end

---Get comprehensive statistics about navmesh pathfinding caches.
---@return table stats Table containing cache statistics with fields: path_cache, distance_cache
function pathfinder.navmesh_get_stats() end

return pathfinder
