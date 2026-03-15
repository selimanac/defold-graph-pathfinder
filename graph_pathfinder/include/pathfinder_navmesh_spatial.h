/**
 * @file pathfinder_navmesh_spatial.h
 * @brief Spatial grid index for fast cell lookup in navigation meshes
 *
 * This module implements a grid-based spatial index for accelerating point-in-cell
 * queries on navigation meshes. Instead of testing every cell for point containment
 * (O(N) complexity), the spatial index provides O(1) grid cell lookup followed by
 * testing only cells that overlap the grid cell.
 *
 * Algorithm Overview:
 * 1. Partition 2D space into uniform grid cells
 * 2. For each NavMesh cell, compute bounding box and insert into overlapping grid cells
 * 3. For point queries: hash position to grid cell, test only candidate NavMesh cells
 *
 * Key Features:
 * - O(1) grid cell lookup via position hashing
 * - O(cells_per_grid) point-in-polygon tests (typically 1-3 cells per grid cell)
 * - Automatic grid sizing based on NavMesh bounds and configuration
 * - Ray casting algorithm for point-in-polygon tests
 *
 * Use Cases:
 * - find_cell_at_position() for spawning agents at arbitrary positions
 * - Click-to-move UI for selecting navigation targets
 * - Dynamic object placement on navigation meshes
 *
 * Thread Safety: Not thread-safe. Designed for single-threaded use.
 */

#ifndef PATHFINDER_NAVMESH_SPATIAL_H
#define PATHFINDER_NAVMESH_SPATIAL_H

#include "pathfinder_navmesh_types.h"
#include "pathfinder_types.h"
#include <stdint.h>

namespace pathfinder
{
    namespace navmesh
    {
        namespace spatial
        {
            // Default spatial index configuration
            const float    SPATIAL_INDEX_MIN_CELL_SIZE = 1.0f;
            const float    SPATIAL_INDEX_MAX_CELL_SIZE = 2.0f;
            const uint32_t SPATIAL_INDEX_MAX_GRID_DIM = 1000;

            /*******************************************/
            // SPATIAL INDEX MANAGEMENT
            /*******************************************/

            /**
             * @brief Build spatial index for fast cell lookup
             * @param navmesh NavMesh to build index for
             * @param debug Pass ctx->m_DebugMode to enable diagnostic output
             * @return Pointer to newly created spatial index, NULL on failure
             *
             * Algorithm:
             * 1. Compute NavMesh bounding box (min/max over all cell vertices)
             * 2. Calculate grid cell size (clamped to config min_cell_size/max_cell_size)
             * 3. Create 2D grid array (width × height)
             * 4. For each NavMesh cell:
             *    - Compute bounding box
             *    - Insert into all overlapping grid cells
             *
             * Uses configuration stored in navmesh->m_SpatialConfig.
             *
             * Time Complexity: O(N × V) where N = cells, V = avg vertices per cell
             * Memory: O(grid_width × grid_height + N × overlap_factor)
             *
             * Caller is responsible for calling destroy_spatial_index() when done.
             * Returns NULL if NavMesh has no cells or memory allocation fails.
             */
            NavMeshSpatialIndex* build_spatial_index(PolygonNavMesh* navmesh, bool debug = false);

            /**
             * @brief Destroy spatial index and free all resources
             * @param index Spatial index to destroy (can be NULL)
             *
             * Deallocates grid array and all cell index arrays.
             * Safe to call with NULL pointer (no-op).
             *
             * Time Complexity: O(grid_width × grid_height)
             */
            void destroy_spatial_index(NavMeshSpatialIndex* index);

            /*******************************************/
            // CELL LOOKUP
            /*******************************************/

            /**
             * @brief Find NavMesh cell containing a given position
             * @param navmesh NavMesh to search
             * @param position Position to query
             * @param enable_fallback If true, find nearest cell when position not in any cell (default: true)
             * @param out_used_fallback Output parameter indicating if fallback was used (optional, can be NULL)
             * @param debug Pass ctx->m_DebugMode to enable diagnostic output (default: false)
             * @return Cell index containing position, -1 if not found or fallback disabled
             *
             * Algorithm:
             * 1. Hash position to grid cell coordinates
             * 2. Retrieve list of candidate NavMesh cells in that grid cell
             * 3. Test each candidate with point_in_polygon()
             * 4. Return first match (or -1 if no matches)
             * 5. If no match and enable_fallback=true: Find nearest cell by center distance
             *
             * Time Complexity: O(cells_per_grid_cell × avg_vertices)
             * - Typically O(1) with good spatial distribution
             * - Worst case O(N × V) if all cells overlap one grid cell
             * - Fallback: O(N) when no exact match found
             *
             * Returns -1 if:
             * - Position outside grid bounds
             * - No cells in grid cell at position
             * - Point not inside any candidate cell polygons AND enable_fallback=false
             *
             * Fallback Behavior:
             * - When enable_fallback=true (default): Returns nearest cell and sets *out_used_fallback=true
             * - When enable_fallback=false: Returns -1 and sets *out_used_fallback=false
             *
             * Use Cases:
             * 1. enable_fallback=true: User can click anywhere, always get a path (original behavior)
             * 2. enable_fallback=false: Reject clicks on walls/obstacles (prevent invalid paths)
             * 3. Check out_used_fallback: Move agent to nearest cell vs target position
             */
            int find_cell_at_position(PolygonNavMesh* navmesh,
                                      Vec2            position,
                                      bool            enable_fallback = true,
                                      bool*           out_used_fallback = NULL,
                                      bool            debug = false);

            /*******************************************/
            // UTILITY FUNCTIONS
            /*******************************************/

            /**
             * @brief Compute axis-aligned bounding box of a cell
             * @param cell Cell to compute bounds for
             * @param out_min Output minimum corner (bottom-left)
             * @param out_max Output maximum corner (top-right)
             *
             * Finds min/max x and y coordinates over all cell vertices.
             *
             * Time Complexity: O(vertex_count)
             */
            void get_cell_bounds(Cell* cell, Vec2* out_min, Vec2* out_max);

            /**
             * @brief Test if a point is inside a polygon using ray casting
             * @param point Point to test
             * @param vertices Polygon vertices (any winding order)
             * @param vertex_count Number of vertices
             * @return true if point is inside polygon, false otherwise
             *
             * Ray Casting Algorithm:
             * - Cast horizontal ray from point to +infinity
             * - Count intersections with polygon edges
             * - Odd count = inside, even count = outside
             *
             * Time Complexity: O(vertex_count)
             * Handles edge cases: point on vertex, point on edge (consistent behavior).
             */
            bool point_in_polygon(Vec2 point, Vec2* vertices, uint32_t vertex_count);

        } // namespace spatial
    } // namespace navmesh
} // namespace pathfinder

#endif // PATHFINDER_NAVMESH_SPATIAL_H
