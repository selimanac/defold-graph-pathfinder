/**
 * @file pathfinder_navmesh.h
 * @brief Navigation mesh pathfinding with polygon-based A* and funnel algorithm
 *
 * This module implements polygon-based pathfinding (Recast/Detour-style) with:
 * - Navigation mesh management (add/remove polygonal cells)
 * - Polygon A* algorithm for finding cell corridors
 * - Simple Stupid Funnel Algorithm (SSFA) for path smoothing
 * - Spatial grid index for fast cell lookup at arbitrary positions
 * - LRU path caching with version-tracked invalidation
 * - Runtime agent radius collision avoidance via portal offsetting
 *
 * Algorithm Pipeline:
 * 1. Build NavMesh: Add cells → Build adjacency graph
 * 2. Pathfinding: Polygon A* (find corridor) → Funnel (extract shortest path)
 * 3. Output: Smooth waypoint path with agent radius collision avoidance
 *
 * Memory Layout:
 * - Cells: Flat array with per-cell vertex and neighbor data
 * - Spatial Index: Grid-based acceleration for O(1) cell lookup
 * - Pathfinding State: Pre-allocated arrays (shared heap pool, distance cache, path cache)
 *
 * Thread Safety: Not thread-safe. Designed for single-threaded use.
 */

#ifndef PATHFINDER_NAVMESH_H
#define PATHFINDER_NAVMESH_H

#include "pathfinder_constants.h"
#include "pathfinder_navmesh_types.h"
#include "pathfinder_types.h"
#include "pathfinder_heap.h"
#include "pathfinder_cache.h"
#include "pathfinder_distance_cache.h"
#include <stdint.h>

namespace pathfinder
{
    namespace navmesh
    {
        /*******************************************/
        // INITIALIZATION & SHUTDOWN
        /*******************************************/

        /**
         * @brief Initialize the NavMesh pathfinding system
         * @param max_cells Maximum number of polygon cells in the navigation mesh
         * @param max_edges_per_cell Maximum edges/neighbors per cell
         * @param pool_block_size Heap pool block size for A* algorithm (default: 32, automatically clamped to max_cells if larger)
         * @param min_cell_size Minimum spatial index grid cell size (default: 1.0f)
         * @param max_cell_size Maximum spatial index grid cell size (default: 2.0f)
         * @param max_grid_dim Maximum spatial index grid dimension (default: 1000)
         * @param cache_size Number of paths to cache (default: 32, set to 0 to disable caching)
         * @param max_cache_path_length Maximum length of cached paths in cells (default: 64)
         *
         * Allocates all memory upfront for cells, adjacency, spatial index, and pathfinding state.
         * Also initializes heap pool, path cache, and distance cache subsystems.
         *
         * IMPORTANT: The heap pool capacity equals max_cells. If pool_block_size > max_cells,
         * it will be automatically clamped to max_cells to prevent heap allocation failures.
         * Recommended: Use pool_block_size = 32 (default) for most navigation meshes.
         * Use larger values (64-128) for very dense meshes or long paths.
         *
         * CACHE: Path caching provides 10-100× speedup for repeated paths (e.g., tower defense).
         * - Set cache_size=0 to disable (zero overhead)
         * - Recommended cache_size=32-128 for typical games
         * - Paths invalidated automatically on NavMesh changes (via version tracking)
         *
         * Time Complexity: O(max_cells + spatial_grid_size)
         * Memory: O(max_cells * (vertices + neighbors) + spatial_grid_size)
         *
         * Must be called before any other navmesh operations.
         */
        void init(uint32_t max_cells,
                  uint32_t max_edges_per_cell,
                  uint32_t pool_block_size = 32,
                  float    min_cell_size = 1.0f,
                  float    max_cell_size = 2.0f,
                  uint32_t max_grid_dim = 1000,
                  uint32_t cache_size = 32,
                  uint32_t max_cache_path_length = 64);

        /**
         * @brief Shutdown and cleanup the NavMesh system
         *
         * Deallocates all memory and resets version counters.
         * All cell IDs become invalid after this call.
         *
         * Time Complexity: O(1)
         */
        void shutdown();

        /*******************************************/
        // FUNNEL ALGORITHM CONFIGURATION
        /*******************************************/

        /**
         * @brief Initialize funnel algorithm configuration with custom tolerances
         * @param portal_vertex_tolerance Tolerance for vertex matching in portal extraction (default: 0.002)
         * @param portal_collapse_threshold Threshold for collapsing narrow portals (default: 0.1)
         * @param waypoint_duplicate_tolerance Tolerance for duplicate waypoint filtering (default: 0.001)
         *
         * Must be called AFTER navmesh::init() to customize funnel algorithm tolerances.
         * If not called, default values are used.
         *
         * WHEN TO CUSTOMIZE:
         * - Large world scales (e.g., 1 unit = 1 kilometer): Increase tolerances
         * - Small world scales (e.g., 1 unit = 1 centimeter): Decrease tolerances
         * - Portal extraction failures: Increase portal_vertex_tolerance
         * - Need higher precision: Decrease waypoint_duplicate_tolerance
         *
         * Time Complexity: O(1)
         *
         * Example:
         * @code
         * navmesh::init(100, 8, 32);
         * navmesh::funnel_init(0.005f, 0.2f, 0.01f);  // Custom tolerances for large worlds
         * @endcode
         */
        void funnel_init(float portal_vertex_tolerance = 0.002f,
                         float portal_collapse_threshold = 0.1f,
                         float waypoint_duplicate_tolerance = 0.001f);

        /*******************************************/
        // CELL MANAGEMENT
        /*******************************************/

        /**
         * @brief Add a polygon cell to the navigation mesh
         * @param vertices Array of polygon vertices (must be counter-clockwise or clockwise)
         * @param vertex_count Number of vertices (typically 3 for triangles, but supports general convex polygons)
         * @param status Output parameter for operation status (optional)
         * @return Cell ID (0 to max_cells-1) on success, ERROR (UINT32_MAX) on failure
         *
         * Creates a new walkable polygon cell with computed centroid.
         * Cell IDs are assigned sequentially and remain stable until removed.
         *
         * Time Complexity: O(vertex_count) for centroid calculation
         * Success: status = SUCCESS, returns valid cell ID
         * Failure: status = ERROR_NODE_FULL if no slots available
         *
         * Note: Does not automatically create adjacency edges. Use build_adjacency() or
         * add_edge_manual() to connect cells after adding all polygons.
         */
        uint32_t add_cell(Vec2* vertices, uint32_t vertex_count, PathStatus* status);

        /**
         * @brief Remove a cell from the navigation mesh
         * @param cell_id ID of cell to remove
         *
         * Marks cell as unwalkable and removes all edges connected to/from this cell.
         * Invalidates cached paths containing this cell.
         * Cell ID slot becomes available for reuse via add_cell().
         *
         * Time Complexity: O(max_cells * max_edges_per_cell) - must scan all edges
         * Does nothing if cell ID is invalid or already removed.
         */
        void remove_cell(uint32_t cell_id);

        /**
         * @brief Get the center position of a cell
         * @param cell_id Cell ID to query
         * @return Vec2 centroid position of the cell
         *
         * Returns the pre-computed centroid (average of all vertices).
         *
         * Time Complexity: O(1)
         * Warning: No bounds checking. Ensure cell_id is valid.
         */
        Vec2 get_cell_center(uint32_t cell_id);

        /*******************************************/
        // ADJACENCY & GRAPH BUILDING
        /*******************************************/

        /**
         * @brief Build adjacency graph from cell edges using edge hashing
         *
         * Automatically detects shared edges between cells by hashing edge vertex pairs.
         * Creates bidirectional adjacency links for all pairs of cells sharing an edge.
         *
         * Algorithm:
         * 1. For each cell, hash all edges (vertex pairs)
         * 2. Match edges with identical hash values
         * 3. Create bidirectional neighbor links for matches
         *
         * Time Complexity: O(max_cells * avg_vertices_per_cell)
         * Must be called after adding all cells but before pathfinding.
         *
         * Note: Replaces any existing adjacency data.
         */
        void build_adjacency();

        /**
         * @brief Manually add edge between two cells
         * @param cell1_id First cell ID
         * @param cell2_id Second cell ID
         *
         * Creates a bidirectional adjacency link between two cells.
         * Useful for manually connecting cells or fixing adjacency after modifications.
         *
         * Time Complexity: O(1)
         * Does nothing if either cell is invalid or edge already exists.
         */
        void add_edge_manual(uint32_t cell1_id, uint32_t cell2_id);

        /**
         * @brief Remove edge between two cells
         * @param cell1_id First cell ID
         * @param cell2_id Second cell ID
         *
         * Removes bidirectional adjacency link between cells.
         * Invalidates cached paths using this edge.
         *
         * Time Complexity: O(max_edges_per_cell) - must search neighbors
         * Does nothing if edge doesn't exist or cells are invalid.
         */
        void remove_edge(uint32_t cell1_id, uint32_t cell2_id);

        /*******************************************/
        // PATHFINDING OPERATIONS
        /*******************************************/

        /**
         * @brief Find smoothed path through navigation mesh (Polygon A* + Funnel Algorithm)
         * @param start_cell Starting cell ID
         * @param goal_cell Goal cell ID
         * @param start_pos Starting position within start_cell
         * @param goal_pos Goal position within goal_cell
         * @param out_smooth_path Output array for smoothed waypoints (Vec2 positions)
         * @param max_length Maximum allowed path length in waypoints
         * @param agent_radius Radius of agent for collision avoidance (0 = no offset, >0 = offset portals inward)
         * @param status Output parameter for operation status (optional)
         * @return Number of waypoints in smooth path, 0 on failure
         *
         * Algorithm Pipeline:
         * 1. Polygon A*: Find corridor of adjacent cells from start_cell to goal_cell
         * 2. Portal Extraction: Extract shared edges (portals) between consecutive cells
         * 3. Portal Offsetting: If agent_radius > 0, offset portals inward for collision avoidance
         * 4. Funnel Algorithm: Apply SSFA to find optimal shortest path through portals
         *
         * Time Complexity: O((C + E) log C + P) where:
         * - C = cells, E = edges for Polygon A*
         * - P = portals for Funnel algorithm
         * - Cache hit: O(1)
         *
         * Success Cases:
         * - status = SUCCESS, returns waypoint count > 0
         * - out_smooth_path contains Vec2 waypoints from start_pos to goal_pos
         * - Path is optimal (shortest path through polygon corridor)
         * - Result cached for future queries with same cells
         *
         * Failure Cases:
         * - status = ERROR_START_NODE_INVALID: start_cell invalid or unwalkable
         * - status = ERROR_GOAL_NODE_INVALID: goal_cell invalid or unwalkable
         * - status = ERROR_NO_PATH: no cell corridor exists between cells
         * - status = ERROR_HEAP_FULL: heap pool exhausted during A*
         *
         * Notes:
         * - Agent radius offset provides runtime collision avoidance
         * - Narrow portals (< agent_radius * collapse_threshold) collapse to midpoint
         * - out_smooth_path array grows automatically if needed
         * - max_length is advisory, not strictly enforced
         */
        uint32_t find_path_smoothed(uint32_t       start_cell,
                                    uint32_t       goal_cell,
                                    Vec2           start_pos,
                                    Vec2           goal_pos,
                                    dmArray<Vec2>* out_smooth_path,
                                    uint32_t       max_length,
                                    float          agent_radius,
                                    PathStatus*    status);

        /**
         * @brief Find cell containing a given position using spatial index
         * @param position Position to query
         * @return Cell ID containing position, ERROR (UINT32_MAX) if not found
         *
         * Uses spatial grid index for O(1) grid cell lookup, then performs
         * point-in-polygon tests on candidate cells.
         *
         * Time Complexity: O(cells_per_grid_cell) typically O(1) with good spatial distribution
         * Returns ERROR if position not inside any walkable cell.
         */
        uint32_t find_cell_at_position(Vec2 position);

        /**
         * @brief Find raw cell corridor path without smoothing (Polygon A* only)
         * @param start_cell Starting cell ID
         * @param goal_cell Goal cell ID
         * @param start_pos Starting position within start_cell (used for heuristic)
         * @param goal_pos Goal position within goal_cell (used for heuristic)
         * @param out_cell_path Output array for cell IDs
         * @param max_length Maximum allowed path length in cells
         * @param status Output parameter for operation status (optional)
         * @return Number of cells in path, 0 on failure
         *
         * Runs Polygon A* only (no funnel smoothing). Useful for:
         * - Debugging pathfinding behavior
         * - Custom path smoothing implementations
         * - Visualization of cell corridors
         *
         * Time Complexity: O((C + E) log C) for Polygon A*
         * - Cache hit: O(1)
         *
         * Success: status = SUCCESS, returns cell count > 0
         * Failure: Same error codes as find_path_smoothed()
         *
         * Note: Path is cached and can be reused by find_path_smoothed()
         */
        uint32_t find_path_raw(uint32_t           start_cell,
                               uint32_t           goal_cell,
                               Vec2               start_pos,
                               Vec2               goal_pos,
                               dmArray<uint32_t>* out_cell_path,
                               uint32_t           max_length,
                               PathStatus*        status);

        /*******************************************/
        // VISUALIZATION / DEBUG
        /*******************************************/

        /**
         * @brief Get number of cells in the navigation mesh
         * @return Number of active cells
         *
         * Time Complexity: O(1)
         */
        uint32_t get_cell_count();

        /**
         * @brief Get direct access to cell array for visualization (read-only)
         * @return Pointer to internal cell array (do not modify!)
         *
         * Provides read-only access to cell data for debug rendering.
         * WARNING: Direct memory access - ensure you don't write to returned array.
         *
         * Time Complexity: O(1)
         */
        Cell* get_cells();

        /**
         * @brief Get heap context for advanced usage (read-only)
         * @return Pointer to internal heap context (do not modify!)
         *
         * Useful for debugging heap allocation and performance profiling.
         *
         * Time Complexity: O(1)
         */
        heap::HeapContext* get_heap_context();

        /**
         * @brief Get spatial index for visualization (read-only)
         * @return Pointer to internal spatial index (do not modify!), NULL if not initialized
         *
         * Useful for debug rendering of spatial grid cells.
         *
         * Time Complexity: O(1)
         */
        NavMeshSpatialIndex* get_spatial_index();

        /**
         * @brief Get cache context for statistics/testing (read-only)
         * @return Pointer to internal cache context (do not modify!), NULL if caching disabled
         *
         * Use cache::get_hit_rate() and related functions to query cache performance.
         *
         * Time Complexity: O(1)
         */
        cache::CacheContext* get_cache_context();

        /**
         * @brief Get distance cache context for statistics/testing (read-only)
         * @return Pointer to internal distance cache context (do not modify!), NULL if not initialized
         *
         * Use distance_cache functions to query cache statistics.
         *
         * Time Complexity: O(1)
         */
        distance_cache::DistanceCacheContext* get_distance_cache_context();

        /**
         * @brief Get funnel algorithm configuration (read-only)
         * @return Const pointer to internal funnel configuration
         *
         * Returns the current funnel tolerances (set via funnel_init() or defaults).
         *
         * Time Complexity: O(1)
         */
        const FunnelConfig* get_funnel_config();

    } // namespace navmesh
} // namespace pathfinder

#endif // PATHFINDER_NAVMESH_H
