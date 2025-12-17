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
         * @brief Configuration for spatial index behavior
         *
         * Controls spatial grid partitioning parameters and enables
         * fine-tuning for different graph sizes and densities.
         */
        struct SpatialIndexConfig
        {
            uint32_t m_MaxGridSize;         ///< Maximum grid dimension (default: 1000)
            float    m_MinCellSize;         ///< Minimum cell size (default: 10.0f)
            float    m_MaxCellSize;         ///< Maximum cell size (default: 500.0f)
            uint32_t m_MaxCellSearchRadius; ///< Search radius in cells (default: 1 = 3×3 grid)
            bool     m_Enabled;             ///< Enable/disable spatial index (default: auto based on node count)

            /**
             * @brief Create default configuration
             * @return SpatialIndexConfig with recommended default values
             */
            static SpatialIndexConfig create_default()
            {
                SpatialIndexConfig config;
                config.m_MaxGridSize = 1000;
                config.m_MinCellSize = 10.0f;
                config.m_MaxCellSize = 500.0f;
                config.m_MaxCellSearchRadius = 1;
                config.m_Enabled = true; // Auto-enable based on node count
                return config;
            }
        };

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
         * @brief Spatial index context - encapsulates all spatial index state
         *
         * Context-based architecture allows multiple independent spatial index instances
         * to coexist without interference. Supports ONLY user-managed contexts.
         *
         * Features:
         * - Independent grid storage per context
         * - Separate graph references per context
         * - No global state interference
         * - Allows simultaneous spatial indexing operations
         */
        typedef struct SpatialIndexContext
        {
            // User configuration
            SpatialIndexConfig m_Config;

            // Grid configuration
            float    m_CellSize;
            Vec2     m_GridMin;
            Vec2     m_GridMax;
            uint32_t m_GridWidth;
            uint32_t m_GridHeight;
            bool     m_Initialized;

            // Grid storage - flat arrays for cell edge data
            dmArray<uint32_t>    m_CellEdgeData;  // Flat storage: all edge indices
            dmArray<uint32_t>    m_CellEdgeStart; // Per-cell: start index into m_CellEdgeData
            dmArray<uint32_t>    m_CellEdgeCount; // Per-cell: number of edges
            dmArray<SpatialEdge> m_SpatialEdges;

            // References to graph data (not owned)
            const dmArray<Node>*     m_Nodes;
            const dmArray<Edge>*     m_Edges;
            const dmArray<uint32_t>* m_EdgesIndex;
            const dmArray<uint32_t>* m_EdgeCount;
            const dmArray<bool>*     m_NodeActive;
        } SpatialIndexContext;

        /*******************************************/
        // CONTEXT MANAGEMENT
        /*******************************************/

        /**
         * @brief Create a new spatial index context
         * @return Pointer to newly created context, or NULL on failure
         *
         * Creates a user-managed context with its own independent state.
         * User is responsible for calling destroy_context() when done.
         */
        SpatialIndexContext* create_context();

        /**
         * @brief Destroy a spatial index context and free all resources
         * @param ctx Context to destroy
         *
         * Releases all memory associated with the context.
         */
        void destroy_context(SpatialIndexContext* ctx);

        /*******************************************/
        // SPATIAL INDEX OPERATIONS
        /*******************************************/

        /**
         * @brief Initialize the spatial index system
         * @param ctx Context to initialize
         * @param nodes Pointer to node array
         * @param edges Pointer to edge array
         * @param edges_index Pointer to edge index array (per-node edge start)
         * @param edge_count Pointer to edge count array (per-node edge count)
         * @param node_active Pointer to node active flags
         * @param config Configuration for spatial index (use SpatialIndexConfig::create_default() for defaults)
         * @param cell_size Optional: Grid cell size (0 = auto-calculate from edges)
         *
         * Builds the spatial grid index from the current graph state.
         * Automatically calculates grid bounds and cell size from the graph.
         *
         * Auto-calculation strategy for cell_size:
         * - Compute average edge length across all active edges
         * - Use 2× average length as cell size (good balance)
         * - Minimum/Maximum cell sizes controlled by config
         *
         * Grid sizing:
         * - Grid bounds: Minimum bounding box of all nodes
         * - Grid dimensions: ceil(bounds / cell_size)
         * - Maximum grid size controlled by config
         *
         * Time Complexity: O(E) to build edge-to-cell mappings
         * Memory Usage: ~24 bytes per edge + (grid_width × grid_height × 8) bytes
         */
        void init(SpatialIndexContext* ctx, const dmArray<Node>* nodes, const dmArray<Edge>* edges, const dmArray<uint32_t>* edges_index, const dmArray<uint32_t>* edge_count, const dmArray<bool>* node_active, const SpatialIndexConfig& config, float cell_size = 0.0f);

        /**
         * @brief Rebuild the spatial index from current graph state
         * @param ctx Context to rebuild
         *
         * Clears and rebuilds the entire spatial index. Use this after
         * significant graph changes (many nodes/edges added/removed).
         *
         * For incremental updates (single node move), use invalidate_node()
         * and update_node() instead for better performance.
         *
         * Time Complexity: O(E)
         */
        void rebuild(SpatialIndexContext* ctx);

        /**
         * @brief Find the nearest edge to a query position using spatial index
         * @param ctx Context to use
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
        bool query_nearest_edge(SpatialIndexContext* ctx, const Vec2& position, uint32_t* out_from, uint32_t* out_to, Vec2* out_projection, const dmArray<Node>* nodes, const dmArray<bool>* node_active);

        /**
         * @brief Update spatial index when a node moves
         * @param ctx Context to use
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
        void update_node_position(SpatialIndexContext* ctx, uint32_t node_id, const Vec2& old_pos, const Vec2& new_pos);

        /**
         * @brief Invalidate spatial index entries for a node
         * @param ctx Context to use
         * @param node_id Node that was removed or became inactive
         *
         * Removes all edges connected to the specified node from the spatial index.
         * Called when a node is removed or deactivated.
         *
         * Time Complexity: O(degree(node))
         */
        void invalidate_node(SpatialIndexContext* ctx, uint32_t node_id);

        /**
         * @brief Add an edge to the spatial index
         * @param ctx Context to use
         * @param from Source node ID
         * @param to Destination node ID
         * @param bidirectional Whether edge is bidirectional
         *
         * Adds a newly created edge to the spatial index.
         * Computes edge bounds and assigns to appropriate grid cells.
         *
         * Time Complexity: O(1) average case, O(cells_spanned) worst case
         */
        void add_edge(SpatialIndexContext* ctx, uint32_t from, uint32_t to, bool bidirectional);

        /**
         * @brief Remove an edge from the spatial index
         * @param ctx Context to use
         * @param from Source node ID
         * @param to Destination node ID
         *
         * Removes an edge from all grid cells that contain it.
         *
         * Time Complexity: O(cells_spanned)
         */
        void remove_edge(SpatialIndexContext* ctx, uint32_t from, uint32_t to);

        /**
         * @brief Clear the spatial index
         * @param ctx Context to clear
         *
         * Removes all edges from the grid and resets state.
         * Grid structure remains allocated for reuse.
         *
         * Time Complexity: O(grid_width × grid_height)
         */
        void clear(SpatialIndexContext* ctx);

        /**
         * @brief Shutdown and deallocate spatial index system
         * @param ctx Context to shutdown
         *
         * Releases all memory used by the spatial index.
         * After calling this, init() must be called again before using the index.
         *
         * Time Complexity: O(grid_width × grid_height)
         */
        void shutdown(SpatialIndexContext* ctx);

        /**
         * @brief Check if spatial index is initialized and ready
         * @param ctx Context to check
         * @return true if initialized, false otherwise
         */
        bool is_initialized(SpatialIndexContext* ctx);

        /**
         * @brief Get spatial index statistics
         * @param ctx Context to query
         * @param cell_count Output: Total number of grid cells (can be NULL)
         * @param edge_count Output: Total edges in index (can be NULL)
         * @param avg_edges_per_cell Output: Average edges per cell (can be NULL)
         * @param max_edges_per_cell Output: Maximum edges in any cell (can be NULL)
         *
         * Retrieves statistics for monitoring and tuning the spatial index.
         *
         * Time Complexity: O(grid_width × grid_height) to compute stats
         */
        void get_stats(SpatialIndexContext* ctx, uint32_t* cell_count, uint32_t* edge_count, float* avg_edges_per_cell, uint32_t* max_edges_per_cell);

        /**
         * @brief Debug information for visualization
         *
         * Contains spatial index debug data for rendering grid overlays,
         * cell boundaries, and edge distributions.
         */
        struct DebugInfo
        {
            Vec2     m_GridMin;         ///< Grid bottom-left corner (world space)
            Vec2     m_GridMax;         ///< Grid top-right corner (world space)
            float    m_CellSize;        ///< Size of each cell (world units)
            uint32_t m_GridWidth;       ///< Grid width in cells
            uint32_t m_GridHeight;      ///< Grid height in cells
            uint32_t m_TotalCells;      ///< Total number of cells
            uint32_t m_TotalEdges;      ///< Total unique edges
            float    m_AvgEdgesPerCell; ///< Average edges per cell
            uint32_t m_MaxEdgesPerCell; ///< Maximum edges in any cell
            bool     m_Initialized;     ///< Whether spatial index is active
        };

        /**
         * @brief Get debug information for visualization
         * @param ctx Context to query
         * @param out_info Output: Debug info structure
         * @return true if info retrieved successfully, false if context is NULL or not initialized
         *
         * Retrieves debug information for rendering grid overlays and diagnostics.
         * Useful for visualizing spatial partitioning and identifying hotspots.
         *
         * Example usage:
         * @code
         * DebugInfo info;
         * if (get_debug_info(ctx, &info)) {
         *     // Draw grid cells
         *     for (uint32_t y = 0; y < info.m_GridHeight; ++y) {
         *         for (uint32_t x = 0; x < info.m_GridWidth; ++x) {
         *             Vec2 cell_min(info.m_GridMin.x + x * info.m_CellSize,
         *                          info.m_GridMin.y + y * info.m_CellSize);
         *             Vec2 cell_max(cell_min.x + info.m_CellSize,
         *                          cell_min.y + info.m_CellSize);
         *             draw_rectangle(cell_min, cell_max, ...);
         *         }
         *     }
         * }
         * @endcode
         */
        bool get_debug_info(SpatialIndexContext* ctx, DebugInfo* out_info);

    } // namespace spatial_index
} // namespace pathfinder

#endif
