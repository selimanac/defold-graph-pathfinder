/**
 * @file pathfinder_navmesh_astar.h
 * @brief Polygon-based A* pathfinding for navigation meshes (Recast-style)
 *
 * This module implements A* pathfinding on a polygon graph derived from a navigation mesh.
 * Unlike traditional node-based A* which finds paths through waypoint nodes, polygon A*
 * finds paths through adjacent polygonal cells.
 *
 * Algorithm Overview:
 * 1. Build polygon graph from NavMesh (cells become nodes, shared edges become connections)
 * 2. Run A* on polygon graph to find corridor of adjacent cells
 * 3. Result is a sequence of cell IDs (corridor) that connects start to goal
 *
 * Key Differences from Node-Based A*:
 * - Nodes represent entire polygons (cells), not point locations
 * - Heuristic uses portal midpoints (shared edge centers) for distance estimation
 * - Path is a corridor of cells, not waypoint positions
 * - Requires funnel algorithm to extract actual waypoint path
 *
 * This implementation is compatible with Recast/Detour navigation mesh structure
 * but uses flat array data structures for performance.
 *
 * Thread Safety: Not thread-safe. Designed for single-threaded use.
 */

#ifndef PATHFINDER_NAVMESH_ASTAR_H
#define PATHFINDER_NAVMESH_ASTAR_H

#include "dmarray_include.h"
#include "pathfinder_constants.h"
#include "pathfinder_navmesh_types.h"
#include "pathfinder_types.h"
#include "pathfinder_heap.h"
#include "pathfinder_cache.h"
#include <stdint.h>

namespace pathfinder
{
    // Forward declaration for distance_cache
    namespace distance_cache
    {
        struct DistanceCacheContext;
    }

    namespace navmesh
    {
        namespace astar
        {
            /*******************************************/
            // POLYGON GRAPH BUILDING
            /*******************************************/

            /**
             * @brief Build polygon graph from navigation mesh for A* pathfinding
             * @param navmesh NavMesh containing cells and adjacency data
             * @param polygon_graph Output array for polygon nodes (cleared and populated)
             * @param config Funnel configuration for portal extraction tolerances
             *
             * Converts NavMesh cell structure into polygon graph format:
             * - Each cell becomes a PolygonNode
             * - Shared edges between cells identified via portal extraction
             * - Portal midpoints computed for heuristic distance calculations
             *
             * Time Complexity: O(max_cells * avg_neighbors * avg_vertices)
             * Must be called before find_polygon_path().
             *
             * Note: polygon_graph is cleared and resized to match cell count.
             */
            void build_polygon_graph(PolygonNavMesh* navmesh, dmArray<PolygonNode>& polygon_graph, const FunnelConfig* config);

            /*******************************************/
            // POLYGON A* PATHFINDING
            /*******************************************/

            /**
             * @brief Find cell corridor path through polygon graph using A* algorithm
             * @param navmesh NavMesh containing cells
             * @param polygon_graph Polygon graph nodes (from build_polygon_graph)
             * @param start_cell Starting cell index
             * @param goal_cell Goal cell index
             * @param start_pos Starting position within start_cell (for heuristic)
             * @param goal_pos Goal position within goal_cell (for heuristic)
             * @param out_cell_path Output array for cell IDs (corridor from start to goal)
             * @param max_length Maximum allowed path length in cells
             * @param status Output parameter for operation status (optional)
             * @param heap_ctx Optional heap context (NULL = use global default context)
             * @param dist_cache_ctx Optional distance cache context (NULL = no caching)
             * @param cache_ctx Optional path cache context (NULL = no caching)
             * @param astar_scratch Optional pre-allocated scratch node array sized to max_cells.
             *        When provided with astar_generation, avoids a per-call heap allocation.
             * @param astar_generation Pointer to the caller's generation counter.  Incremented
             *        on every call so that stale scratch entries are detected without a full reset.
             * @return Number of cells in corridor, 0 on failure
             *
             * A* Algorithm on Polygon Graph:
             * - Heuristic: Euclidean distance from portal midpoints to goal_pos
             * - Cost: Euclidean distance between consecutive portal midpoints
             * - Priority queue: Min-heap with object pooling
             * - Optimization: Checks path cache first for O(1) retrieval
             *
             * Time Complexity: O((C + E) log C) where C=cells, E=edges
             * - Cache hit: O(1)
             * - Cache miss: Full A* search
             *
             * Success Cases:
             * - status = SUCCESS, returns cell count > 0
             * - out_cell_path contains cell IDs from start_cell to goal_cell
             * - Path is optimal corridor (minimal cost through cells)
             * - Result cached if cache_ctx provided
             *
             * Failure Cases:
             * - status = ERROR_START_NODE_INVALID: start_cell invalid or unwalkable
             * - status = ERROR_GOAL_NODE_INVALID: goal_cell invalid or unwalkable
             * - status = ERROR_NO_PATH: no corridor exists between cells
             * - status = ERROR_HEAP_FULL: heap pool exhausted
             *
             * Notes:
             * - out_cell_path array grows automatically if needed
             * - max_length is advisory, not strictly enforced
             * - Result is cell corridor, not waypoint positions (use funnel algorithm)
             * - Cache contexts are optional for advanced performance tuning
             */
            uint32_t find_polygon_path(PolygonNavMesh*                       navmesh,
                                       dmArray<PolygonNode>&                 polygon_graph,
                                       uint32_t                              start_cell,
                                       uint32_t                              goal_cell,
                                       Vec2                                  start_pos,
                                       Vec2                                  goal_pos,
                                       dmArray<uint32_t>*                    out_cell_path,
                                       uint32_t                              max_length,
                                       PathStatus*                           status,
                                       heap::HeapContext*                    heap_ctx = NULL,
                                       distance_cache::DistanceCacheContext* dist_cache_ctx = NULL,
                                       cache::CacheContext*                  cache_ctx = NULL,
                                       PolygonAStarNode*                     astar_scratch = NULL,
                                       uint32_t*                             astar_generation = NULL);

        } // namespace astar
    } // namespace navmesh
} // namespace pathfinder

#endif // PATHFINDER_NAVMESH_ASTAR_H
