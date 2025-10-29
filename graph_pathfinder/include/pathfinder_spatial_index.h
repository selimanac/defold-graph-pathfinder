#ifndef PATHFINDER_SPATIAL_INDEX_H
#define PATHFINDER_SPATIAL_INDEX_H

#include "dmarray_include.h"
#include "pathfinder_types.h"
#include <cstdint>

/**
 * @file pathfinder_spatial_index.h
 * @brief Spatial grid index for accelerating edge projection queries
 *
 * This module provides a 2D spatial grid index to accelerate finding the nearest
 * edge to a query position. Instead of O(V×E) full scan, the spatial index provides
 * O(1) average case lookups by partitioning edges into grid cells.
 *
 * Features:
 * - Grid-based spatial partitioning with configurable cell size
 * - Per-cell edge lists for fast spatial queries
 * - Automatic bounds calculation and grid sizing
 * - Support for dynamic graph updates (add/remove/move nodes)
 * - Version tracking for cache invalidation
 * - Configurable neighbor search radius
 *
 * Performance:
 * - Initialization: O(E) - build edge-to-cell mappings
 * - Query: O(k) where k = edges in nearby cells (typically << E)
 * - Update: O(1) per edge move/add/remove
 * - Memory: ~16-24 bytes per edge + grid overhead
 *
 * Usage Pattern:
 * 1. Call init() once after graph construction
 * 2. Call rebuild() after significant graph changes
 * 3. Use query_nearest_edge() instead of O(V×E) scan
 * 4. Call invalidate_node() when nodes move
 * 5. Call shutdown() to cleanup
 *
 * When to Use:
 * - Graph size: >500 nodes recommended
 * - Projection frequency: >20 projections/frame
 * - Expected speedup: 10-100× for large graphs
 *
 * Design Notes:
 * - Grid cells are fixed-size (not adaptive)
 * - Cell size auto-calculated from average edge length
 * - Edges may span multiple cells (stored in all)
 * - Fallback to full scan if spatial index disabled
 */

namespace pathfinder
{
    namespace spatial_index
    {
        /**
         * @brief Edge stored in spatial grid with geometric bounds
         *
         * Stores edge connectivity and spatial bounds (AABB) for efficient
         * grid cell assignment and spatial queries.
         */
        struct SpatialEdge
        {
            uint32_t m_From;     ///< Source node ID
            uint32_t m_To;       ///< Destination node ID
            Vec2     m_MinBound; ///< Minimum corner of axis-aligned bounding box
            Vec2     m_MaxBound; ///< Maximum corner of axis-aligned bounding box
        };

        /**
         * @brief Grid cell edge storage using flat arrays
         *
         * Cells are indexed by cell_id = grid_y * grid_width + grid_x.
         * Cell's edges are stored in m_CellEdgeData from index m_CellEdgeStart[cell_id]
         * to index m_CellEdgeStart[cell_id] + m_CellEdgeCount[cell_id] - 1.
         *
         * This flat array design is compatible with dmArray (no copy constructor needed)
         * and provides better cache locality than nested data structures.
         */

        /**
         * @brief Initialize the spatial index system
         * @param nodes Pointer to node array
         * @param edges Pointer to edge array
         * @param edges_index Pointer to edge index array (per-node edge start)
         * @param edge_count Pointer to edge count array (per-node edge count)
         * @param node_active Pointer to node active flags
         * @param cell_size Optional: Grid cell size (0 = auto-calculate from edges)
         *
         * Builds the spatial grid index from the current graph state.
         * Automatically calculates grid bounds and cell size from the graph.
         *
         * Auto-calculation strategy for cell_size:
         * - Compute average edge length across all active edges
         * - Use 2× average length as cell size (good balance)
         * - Minimum cell size: 10.0 units
         * - Maximum cell size: 500.0 units
         *
         * Grid sizing:
         * - Grid bounds: Minimum bounding box of all nodes
         * - Grid dimensions: ceil(bounds / cell_size)
         * - Maximum grid size: 1000×1000 cells (prevent excessive memory)
         *
         * Time Complexity: O(E) to build edge-to-cell mappings
         * Memory Usage: ~24 bytes per edge + (grid_width × grid_height × 8) bytes
         */
        void init(const dmArray<Node>* nodes, const dmArray<Edge>* edges, const dmArray<uint32_t>* edges_index, const dmArray<uint32_t>* edge_count, const dmArray<bool>* node_active, float cell_size = 0.0f);

        /**
         * @brief Rebuild the spatial index from current graph state
         *
         * Clears and rebuilds the entire spatial index. Use this after
         * significant graph changes (many nodes/edges added/removed).
         *
         * For incremental updates (single node move), use invalidate_node()
         * and update_node() instead for better performance.
         *
         * Time Complexity: O(E)
         */
        void rebuild();

        /**
         * @brief Find the nearest edge to a query position using spatial index
         * @param position Query position
         * @param out_from Output: Source node ID of nearest edge
         * @param out_to Output: Destination node ID of nearest edge
         * @param out_projection Output: Closest point on nearest edge
         * @param nodes Pointer to node array for position lookup
         * @param node_active Pointer to node active flags
         * @return true if edge found, false if no edges in search area
         *
         * Searches for the nearest edge using the spatial grid index:
         * 1. Convert query position to grid coordinates
         * 2. Check edges in query cell and neighboring cells
         * 3. Project position onto each edge segment
         * 4. Return edge with minimum distance
         *
         * Search radius: Checks 3×3 grid of cells centered on query position
         * (configurable via MAX_CELL_SEARCH_RADIUS)
         *
         * Time Complexity: O(k) where k = edges in nearby cells (typically 10-50)
         * Worst case: O(E) if all edges in search area
         *
         * Special cases:
         * - Returns false if spatial index not initialized
         * - Falls back to full scan if grid is empty
         * - Skips edges with inactive nodes
         */
        bool query_nearest_edge(const Vec2& position, uint32_t* out_from, uint32_t* out_to, Vec2* out_projection, const dmArray<Node>* nodes, const dmArray<bool>* node_active);

        /**
         * @brief Update spatial index when a node moves
         * @param node_id Node that moved
         * @param old_pos Previous position
         * @param new_pos New position
         *
         * Incrementally updates the spatial index when a single node moves.
         * Removes edges from old cells and adds to new cells as needed.
         *
         * More efficient than rebuild() for single node updates.
         *
         * Time Complexity: O(degree(node)) where degree = number of edges
         */
        void update_node_position(uint32_t node_id, const Vec2& old_pos, const Vec2& new_pos);

        /**
         * @brief Invalidate spatial index entries for a node
         * @param node_id Node that was removed or became inactive
         *
         * Removes all edges connected to the specified node from the spatial index.
         * Called when a node is removed or deactivated.
         *
         * Time Complexity: O(degree(node))
         */
        void invalidate_node(uint32_t node_id);

        /**
         * @brief Add an edge to the spatial index
         * @param from Source node ID
         * @param to Destination node ID
         * @param bidirectional Whether edge is bidirectional
         *
         * Adds a newly created edge to the spatial index.
         * Computes edge bounds and assigns to appropriate grid cells.
         *
         * Time Complexity: O(1) average case, O(cells_spanned) worst case
         */
        void add_edge(uint32_t from, uint32_t to, bool bidirectional);

        /**
         * @brief Remove an edge from the spatial index
         * @param from Source node ID
         * @param to Destination node ID
         *
         * Removes an edge from all grid cells that contain it.
         *
         * Time Complexity: O(cells_spanned)
         */
        void remove_edge(uint32_t from, uint32_t to);

        /**
         * @brief Clear the spatial index
         *
         * Removes all edges from the grid and resets state.
         * Grid structure remains allocated for reuse.
         *
         * Time Complexity: O(grid_width × grid_height)
         */
        void clear();

        /**
         * @brief Shutdown and deallocate spatial index system
         *
         * Releases all memory used by the spatial index.
         * After calling this, init() must be called again before using the index.
         *
         * Time Complexity: O(grid_width × grid_height)
         */
        void shutdown();

        /**
         * @brief Check if spatial index is initialized and ready
         * @return true if initialized, false otherwise
         */
        bool is_initialized();

        /**
         * @brief Get spatial index statistics
         * @param cell_count Output: Total number of grid cells (can be NULL)
         * @param edge_count Output: Total edges in index (can be NULL)
         * @param avg_edges_per_cell Output: Average edges per cell (can be NULL)
         * @param max_edges_per_cell Output: Maximum edges in any cell (can be NULL)
         *
         * Retrieves statistics for monitoring and tuning the spatial index.
         *
         * Time Complexity: O(grid_width × grid_height) to compute stats
         */
        void get_stats(uint32_t* cell_count, uint32_t* edge_count, float* avg_edges_per_cell, uint32_t* max_edges_per_cell);

    } // namespace spatial_index
} // namespace pathfinder

#endif
