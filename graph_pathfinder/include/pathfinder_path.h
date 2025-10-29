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

        /**
         * @brief Get edges for a specific node with bidirectionality information
         * @param node_id Node ID to query edges for
         * @param out_edges Output array to store EdgeInfo structures
         * @param include_bidirectional If true (default), returns all edges. If false, returns only unidirectional edges.
         * @param include_incoming If true, includes incoming edges. If false (default), includes only outgoing edges.
         *
         * Retrieves edges from/to the specified node and populates the output array
         * with complete edge information including whether each edge is bidirectional.
         *
         * EdgeInfo Structure:
         * - m_From: Source node ID (may differ from node_id if incoming edges included)
         * - m_To: Destination node ID (may equal node_id if incoming edges included)
         * - m_Cost: Edge traversal cost
         * - m_Bidirectional: True if reverse edge exists
         *
         * Edge Types:
         * - Outgoing: Edges where m_From equals node_id (A->B where A is node_id)
         * - Incoming: Edges where m_To equals node_id (C->A where A is node_id)
         *
         * Bidirectionality Detection:
         * - Checks if reverse edge exists by scanning destination node's edges
         * - Two edges may have different costs but still be considered bidirectional
         * - Only checks existence, not cost symmetry
         *
         * Filtering Behavior:
         * - include_bidirectional=true (default): Returns edges regardless of bidirectionality
         * - include_bidirectional=false: Returns only unidirectional edges (skips bidirectional ones)
         * - include_incoming=false (default): Returns only outgoing edges
         * - include_incoming=true: Returns both outgoing and incoming edges
         *
         * Time Complexity:
         * - Outgoing only: O(E1 * E2) where E1 = edges from node, E2 = avg edges per dest
         * - With incoming: O(N * E * E) where N = total nodes, E = max edges per node
         * - The nested complexity for incoming comes from:
         *   - Outer loop: scan all N nodes in graph
         *   - Inner loop: check E edges per node for matches
         *   - Bidirectional check: scan E edges of destination node
         *
         * Memory:
         * - Caller must pre-allocate out_edges array with sufficient capacity
         * - Recommended capacity: max_edges_per_node (outgoing only) or 2*max_edges_per_node (with incoming)
         * - Array is cleared before populating (SetSize(0))
         *
         * Success Cases:
         * - Returns number of edges found (0 to graph capacity)
         * - out_edges populated with EdgeInfo structures
         * - Array size matches return value
         *
         * Failure Cases:
         * - Returns 0 if node_id is invalid or inactive
         * - Returns 0 if node has no edges (outgoing and/or incoming based on parameters)
         * - out_edges array cleared (size = 0)
         *
         * Example Usage:
         * @code
         * dmArray<EdgeInfo> edges;
         * edges.SetCapacity(max_edges_per_node * 2); // Account for incoming edges
         *
         * // Get outgoing edges only (default)
         * uint32_t count = get_node_edges(node_id, &edges);
         *
         * // Get only unidirectional outgoing edges
         * uint32_t count = get_node_edges(node_id, &edges, false);
         *
         * // Get both outgoing and incoming edges
         * uint32_t count = get_node_edges(node_id, &edges, true, true);
         *
         * // Get only unidirectional edges (both directions)
         * uint32_t count = get_node_edges(node_id, &edges, false, true);
         *
         * for (uint32_t i = 0; i < count; i++) {
         *     printf("Edge %u->%u, cost=%.2f, bidirectional=%s\n",
         *            edges[i].m_From, edges[i].m_To, edges[i].m_Cost,
         *            edges[i].m_Bidirectional ? "true" : "false");
         * }
         * @endcode
         *
         * Notes:
         * - Does not modify graph state (read-only operation)
         * - Thread-safe for concurrent reads (if graph not modified)
         * - Bidirectionality is computed dynamically (not cached)
         * - Including incoming edges requires scanning all nodes (slower)
         */
        uint32_t get_node_edges(const uint32_t node_id, dmArray<EdgeInfo>* out_edges, const bool include_bidirectional = true, const bool include_incoming = false);

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

        /**
         * @brief Find path with exit point projection to arbitrary end position
         * @param start_position Starting position (optional, can be zero Vec2 if using start_node_id)
         * @param end_position Ending position (arbitrary position, not necessarily a node)
         * @param start_node_id Starting node ID (optional, set to INVALID_ID if using start_position)
         * @param out_path Output array to store path node IDs
         * @param max_path Maximum allowed path length for output
         * @param out_entry_point Output: projection point on graph edge from start (optional, can be NULL)
         * @param out_exit_point Output: projection point on graph edge to reach end_position
         * @param status Output parameter for operation status (optional)
         * @param virtual_max_path Maximum path length for virtual node search (default: 64)
         * @return Length of path (number of nodes), 0 if no path found
         *
         * Projected Pathfinding with Exit Point Algorithm:
         * Mode 1 (start_node_id provided, start_position ignored):
         *   1. Find nearest edge to end_position by projecting onto all graph edges
         *   2. Create temporary "virtual exit" node at projection point
         *   3. Connect virtual exit node to edge endpoints with distance-based costs
         *   4. Run A* from start_node_id to virtual exit node
         *   5. Remove virtual exit node and return path (excluding virtual node)
         *   6. out_exit_point contains projection on graph, out_entry_point will be zero
         *
         * Mode 2 (start_node_id = INVALID_ID, start_position provided):
         *   1. Find nearest edge to start_position (entry point projection)
         *   2. Find nearest edge to end_position (exit point projection)
         *   3. Create temporary "virtual entry" node at entry projection
         *   4. Create temporary "virtual exit" node at exit projection
         *   5. Connect both virtual nodes to their respective edge endpoints
         *   6. Run A* from virtual entry to virtual exit
         *   7. Remove both virtual nodes and return path (excluding virtual nodes)
         *   8. out_entry_point and out_exit_point contain both projections
         *
         * Time Complexity: O(V * E_avg * 2 + A*) where:
         * - V * E_avg for entry projection (if Mode 2)
         * - V * E_avg for exit projection
         * - A* for pathfinding between projections
         *
         * Use Cases:
         * - Click-to-move to arbitrary positions (not just nodes)
         * - AI agents moving between dynamic positions
         * - Projectile path prediction
         * - Area-of-effect ability targeting
         *
         * Success Cases:
         * - status = SUCCESS, returns path length > 0
         * - out_path contains node IDs from entry to exit
         * - out_entry_point contains entry projection (Mode 2) or zero (Mode 1)
         * - out_exit_point contains exit projection to reach end_position
         * - Result cached for future queries with same positions
         *
         * Failure Cases:
         * - status = ERROR_START_NODE_INVALID: start_node_id invalid (Mode 1)
         * - status = ERROR_NO_PROJECTION: no edges in graph to project onto
         * - status = ERROR_NODE_FULL: couldn't create virtual node(s)
         * - status = ERROR_EDGE_FULL: couldn't connect virtual node(s)
         * - status = ERROR_NO_PATH: no path between projections
         *
         * Notes:
         * - Set start_node_id to INVALID_ID for Mode 2 (arbitrary start position)
         * - Set start_node_id to valid node ID for Mode 1 (start from node)
         * - Projection considers only active edges between active nodes
         * - Virtual nodes are always cleaned up (even on failure)
         * - virtual_max_path limits search depth (defaults to 64)
         * - out_entry_point can be NULL if not needed
         * - out_exit_point should not be NULL (required output)
         */
        uint32_t find_path_projected_with_exit(const Vec2 start_position, const Vec2 end_position, const uint32_t start_node_id, dmArray<uint32_t>* out_path, const uint32_t max_path, Vec2* out_entry_point, Vec2* out_exit_point, PathStatus* status, const uint32_t virtual_max_path = 64);

    } // namespace path

} // namespace pathfinder
#endif
