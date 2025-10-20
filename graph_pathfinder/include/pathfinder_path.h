#ifndef PATHFINDER_PATH_H
#define PATHFINDER_PATH_H

#include "dmarray_include.h"
#include "pathfinder_constants.h"
#include "pathfinder_types.h"
#include <cstdint>

/**
 * @file pathfinder_path.h
 * @brief A* pathfinding implementation with caching and dynamic graph support
 *
 * This module implements A* pathfinding algorithm with:
 * - Flat array-based graph representation for cache-friendly memory layout
 * - Integrated LRU path cache with version-tracked invalidation
 * - Distance caching for heuristic calculations
 * - Min-heap priority queue with object pooling
 * - Dynamic graph updates (add/remove nodes and edges at runtime)
 * - Projected pathfinding from arbitrary positions
 *
 * Memory Layout:
 * - Nodes: Flat array with active/inactive flags
 * - Edges: Pre-allocated flat array indexed per-node
 * - Pathfinding state: Pre-allocated arrays (g_score, f_score, came_from, closed)
 *
 * Thread Safety: Not thread-safe. Designed for single-threaded use.
 */

namespace pathfinder
{
    namespace path
    {
        /**
         * @brief Initialize the pathfinding system
         * @param max_nodes Maximum number of nodes in the graph
         * @param max_edge_per_node Maximum edges per node
         * @param pool_block_size Heap pool block size for A* algorithm (automatically clamped to max_nodes if larger)
         * @param max_cache_path_length Maximum length of cached paths
         *
         * Allocates all memory upfront for nodes, edges, and pathfinding arrays.
         * Also initializes heap pool, path cache, and distance cache subsystems.
         *
         * IMPORTANT: The heap pool capacity equals max_nodes. If pool_block_size > max_nodes,
         * it will be automatically clamped to max_nodes to prevent heap allocation failures.
         * Recommended: Use pool_block_size = 32 (default) and ensure max_nodes >= 32.
         *
         * Time Complexity: O(max_nodes)
         * Memory: O(max_nodes * max_edge_per_node + max_nodes * 4) for arrays
         *
         * Must be called before any other path operations.
         */
        void init(const uint32_t max_nodes, const uint32_t max_edge_per_node, const uint32_t pool_block_size, const uint32_t max_cache_path_length);

        /**
         * @brief Shutdown and cleanup the pathfinding system
         *
         * Deallocates all memory and resets version counters.
         * All node IDs become invalid after this call.
         *
         * Time Complexity: O(1)
         */
        void shutdown();

        /*******************************************/
        // NODE OPERATIONS
        /*******************************************/

        /**
         * @brief Add a new node to the graph at the specified position
         * @param node_position 2D position of the node
         * @param status Output parameter for operation status (optional)
         * @return Node ID (0 to max_nodes-1) on success, INVALID_ID (UINT32_MAX) on failure
         *
         * Finds first inactive slot and reuses it. Node IDs are stable until removed.
         * Increments node version to invalidate affected cached paths.
         *
         * Time Complexity: O(max_nodes) - linear search for free slot
         * Success: status = SUCCESS, returns valid node ID
         * Failure: status = ERROR_NODE_FULL if no slots available
         *
         * Note: Does not automatically create edges. Use add_edge() to connect nodes.
         */
        uint32_t add_node(const Vec2 node_position, PathStatus* status);

        /**
         * @brief Move an existing node to a new position
         * @param id Node ID to move
         * @param node_position New 2D position for the node
         *
         * Updates node position and invalidates cached paths containing this node.
         * Only updates if position actually changed (uses epsilon comparison).
         * Also invalidates distance cache and connected edges.
         *
         * Time Complexity: O(edge_count) for cache invalidation
         * Safe to call with same position (becomes no-op).
         * Does nothing if node ID is invalid or inactive.
         */
        void move_node(const uint32_t id, const Vec2 node_position);

        /**
         * @brief Remove a node from the graph
         * @param id Node ID to remove
         *
         * Marks node as inactive and removes all edges connected to/from this node.
         * Invalidates cached paths containing this node.
         * Node ID slot becomes available for reuse via add_node().
         *
         * Time Complexity: O(max_nodes * max_edge_per_node) - must scan all edges
         * Does nothing if node ID is invalid or already inactive.
         */
        void remove_node(const uint32_t id);

        /**
         * @brief Get the 2D position of a node
         * @param node_id Node ID to query
         * @return Vec2 position of the node
         *
         * Time Complexity: O(1)
         * Warning: No bounds checking. Ensure node_id is valid.
         */
        Vec2 get_node_position(const uint32_t node_id);

        /*******************************************/
        // EDGE OPERATIONS
        /*******************************************/

        /**
         * @brief Add an edge between two nodes
         * @param from Source node ID
         * @param to Destination node ID
         * @param cost Edge traversal cost (typically distance)
         * @param bidirectional If true, also creates reverse edge (to -> from)
         * @param status Output parameter for operation status (optional)
         *
         * Creates directed edge with specified cost. If bidirectional, creates
         * both directions with same cost. Increments edge version to invalidate
         * affected cached paths.
         *
         * Time Complexity: O(1) for unidirectional, O(2) for bidirectional
         *
         * Errors:
         * - ERROR_START_NODE_INVALID: from node doesn't exist or inactive
         * - ERROR_EDGE_FULL: from node already has max_edge_per_node edges
         * - ERROR_EDGE_FULL: to node full (if bidirectional)
         *
         * Note: Allows duplicate edges (not checked). Multiple edges between
         * same nodes will all be traversed during pathfinding.
         */
        void add_edge(const uint32_t from, const uint32_t to, const float cost, const bool bidirectional, PathStatus* status);

        /**
         * @brief Remove an edge between two nodes
         * @param from Source node ID
         * @param to Destination node ID
         *
         * Removes only the from->to edge. For bidirectional edges, must call
         * twice with reversed parameters. Uses swap-and-pop for O(1) removal.
         * Invalidates cached paths using this edge.
         *
         * Time Complexity: O(edge_count) - must search edges of from node
         * Does nothing if edge doesn't exist or nodes are inactive.
         */
        void remove_edge(const uint32_t from, const uint32_t to);

        /*******************************************/
        // PATHFINDING OPERATIONS
        /*******************************************/

        /**
         * @brief Find optimal path between two nodes using A* algorithm
         * @param start_id Starting node ID
         * @param goal_id Goal node ID
         * @param out_path Output array to store path node IDs (start to goal)
         * @param max_path Maximum allowed path length
         * @param status Output parameter for operation status (optional)
         * @return Length of path (number of nodes), 0 if no path found
         *
         * A* Algorithm Implementation:
         * - Heuristic: Euclidean distance (cached via distance_cache)
         * - Priority queue: Min-heap with object pooling
         * - Optimization: Checks path cache first for O(1) retrieval
         * - Graph changes: Auto-detects via version tracking and restarts if needed
         *
         * Time Complexity: O((V + E) log V) where V=nodes, E=edges
         * - Cache hit: O(1)
         * - Cache miss: Full A* search
         *
         * Memory: Reuses pre-allocated arrays (g_score, f_score, came_from, closed)
         *
         * Success Cases:
         * - status = SUCCESS, returns path length > 0
         * - out_path contains node IDs from start to goal (inclusive)
         * - Path is optimal (minimal cost given edge weights)
         * - Result cached for future queries
         *
         * Failure Cases:
         * - status = ERROR_START_NODE_INVALID: start node doesn't exist or inactive
         * - status = ERROR_GOAL_NODE_INVALID: goal node doesn't exist or inactive
         * - status = ERROR_HEAP_FULL: heap pool exhausted (shouldn't happen if init correct)
         * - status = ERROR_GRAPH_CHANGED: graph modified during search (recursively retries)
         * - status = ERROR_GRAPH_CHANGED_TOO_OFTEN: graph changed too many times (>3 retries)
         * - status = ERROR_NO_PATH: no path exists between nodes
         *
         * Notes:
         * - out_path array grows automatically if needed
         * - max_path is advisory, not strictly enforced
         * - Recursive retry on graph changes with limit of 3 retries (prevents stack overflow)
         */
        uint32_t find_path(const uint32_t start_id, const uint32_t goal_id, dmArray<uint32_t>* out_path, const uint32_t max_path, PathStatus* status);

        /**
         * @brief Find path from arbitrary position to a node
         * @param position Starting position (not necessarily a node)
         * @param goal_id Goal node ID
         * @param out_path Output array to store path node IDs
         * @param max_path Maximum allowed path length for output
         * @param out_entry_point Output: projection point on graph edge (optional)
         * @param status Output parameter for operation status (optional)
         * @param virtual_max_path Maximum path length for virtual node search (default: 64)
         * @return Length of path (number of nodes), 0 if no path found
         *
         * Projected Pathfinding Algorithm:
         * 1. Find nearest edge by projecting position onto all graph edges
         * 2. Create temporary "virtual" node at projection point
         * 3. Connect virtual node to edge endpoints with distance-based costs
         * 4. Run A* from virtual node to goal
         * 5. Remove virtual node and return path (excluding virtual node)
         *
         * Time Complexity: O(V * E_avg + A*) where:
         * - V * E_avg for projection search (all nodes * avg edges per node)
         * - A* for pathfinding from virtual node
         *
         * Use Cases:
         * - Start pathfinding from unit's current position (between nodes)
         * - Click-to-move interfaces
         * - Dynamic spawn points
         *
         * Success Cases:
         * - status = SUCCESS, returns path length > 0
         * - out_path contains node IDs from nearest graph edge to goal
         * - out_entry_point contains the projection point on the graph
         * - Result cached for future queries with same position
         *
         * Failure Cases:
         * - status = ERROR_GOAL_NODE_INVALID: goal doesn't exist or inactive
         * - status = ERROR_NO_PROJECTION: no edges in graph to project onto
         * - status = ERROR_NODE_FULL: couldn't create virtual node
         * - status = ERROR_EDGE_FULL: couldn't connect virtual node
         * - status = ERROR_NO_PATH: no path from projection to goal
         *
         * Notes:
         * - Projection considers only active edges between active nodes
         * - Bidirectional edges checked only once (optimization)
         * - Virtual node is always cleaned up (even on failure)
         * - virtual_max_path limits search depth from virtual node (defaults to 64)
         */
        uint32_t find_path_projected(const Vec2 position, const uint32_t goal_id, dmArray<uint32_t>* out_path, const uint32_t max_path, Vec2* out_entry_point, PathStatus* status, const uint32_t virtual_max_path = 64);

    } // namespace path

} // namespace pathfinder
#endif
