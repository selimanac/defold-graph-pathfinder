/**
 * @file pathfinder_navmesh_types.h
 * @brief Data structures for navigation mesh pathfinding
 *
 * This file defines all core data types used by the NavMesh pathfinding system:
 * - Cell: Polygon representation with vertices and adjacency
 * - PolygonNode: A* graph node for polygon pathfinding
 * - Portal: Shared edge between cells for funnel algorithm
 * - SpatialGridCell: Grid-based spatial index for fast lookups
 * - FunnelConfig: Tolerance configuration for path smoothing
 * - PolygonNavMesh: Main navigation mesh container
 *
 * These types are extracted and refactored from the original raylib_navmesh_buffer.cpp
 * implementation and adapted for use with flat array data structures (dmArray, dmHashTable).
 */

#ifndef PATHFINDER_NAVMESH_TYPES_H
#define PATHFINDER_NAVMESH_TYPES_H

#include "dmarray_include.h"
#include "dmhashtable_include.h"
#include "pathfinder_types.h"
#include <stdbool.h>
#include <stdint.h>

namespace pathfinder
{
    namespace navmesh
    {
        /*******************************************/
        // NAVMESH CELL STRUCTURE (Polygon-based)
        /*******************************************/

        /**
         * @brief Navigation mesh polygon cell
         *
         * Represents a single walkable polygon in the navigation mesh.
         * Typically a triangle (3 vertices) but can be any convex polygon.
         *
         * Memory Layout:
         * - Vertices stored in separate dynamic array (pointed to by m_Vertices)
         * - Neighbor indices stored in separate dynamic array (pointed to by m_NeighborIndices)
         * - Both arrays allocated per-cell during add_cell()
         */
        typedef struct Cell
        {
            Vec2*     m_Vertices;        // Polygon vertices (dynamically allocated, counter-clockwise or clockwise)
            uint32_t  m_VertexCount;     // Number of vertices (typically 3 for triangles)
            Vec2      m_Center;          // Centroid position (pre-computed for heuristic)
            uint32_t  m_NodeId;          // Pathfinding node ID (stable identifier)
            uint32_t* m_NeighborIndices; // Indices of adjacent cells (dynamically allocated)
            uint32_t  m_NeighborCount;   // Number of neighbors (0-8 typically)
            bool      m_Walkable;        // Is walkable? (false = removed/obstacle)
        } Cell;

        /*******************************************/
        // POLYGON GRAPH NODE (for Recast-style A*)
        /*******************************************/

        /**
         * @brief Polygon graph node for A* pathfinding
         *
         * Derived from Cell structure for polygon-based A* algorithm.
         * Each PolygonNode corresponds to a Cell and stores neighbor relationships
         * with pre-computed portal midpoints for heuristic calculations.
         *
         * Note: Max 8 neighbors is a practical limit for typical navmeshes.
         * This can be increased if needed but 8 is sufficient for most geometries.
         */
        typedef struct PolygonNode
        {
            uint32_t m_CellIdx;            // Index of this cell in navmesh
            uint32_t m_NeighborIndices[8]; // Indices of adjacent cells (fixed array)
            Vec2     m_PortalMidpoints[8]; // Midpoint of shared edge with each neighbor (for heuristic)
            uint32_t m_NeighborCount;      // Number of neighbors (0-8)
        } PolygonNode;

        /*******************************************/
        // PORTAL BETWEEN CELLS (for Funnel Algorithm)
        /*******************************************/

        /**
         * @brief Portal (shared edge) between two adjacent cells
         *
         * Used by the funnel algorithm to represent passage between consecutive
         * cells in a polygon corridor. A portal is the shared edge between two
         * neighboring cells.
         *
         * Winding Order:
         * - m_Left and m_Right are oriented relative to direction of travel
         * - When moving from cell A to cell B, portal vertices are ordered
         *   such that the funnel algorithm can maintain left/right boundaries
         */
        typedef struct Portal
        {
            Vec2 m_Left;  // Left portal vertex (in direction of travel)
            Vec2 m_Right; // Right portal vertex (in direction of travel)
            bool m_Valid; // True if portal has valid shared edge, false if degenerate/fallback
        } Portal;

        /*******************************************/
        // SPATIAL GRID STRUCTURES (for fast cell lookup)
        /*******************************************/

        /**
         * @brief Configuration for spatial index grid sizing
         *
         * Controls how the spatial grid is sized when building the index.
         * These values are used to clamp the computed grid cell size.
         *
         * Default Values:
         * - m_MinCellSize: 1.0 (minimum grid cell size in world units)
         * - m_MaxCellSize: 2.0 (maximum grid cell size in world units)
         * - m_MaxGridDim: 1000 (maximum grid dimension to prevent excessive memory)
         *
         * Customization Guidelines:
         * - m_MinCellSize: Should be ~= average NavMesh cell size
         * - m_MaxCellSize: Should be ~= 2-3× average NavMesh cell size
         * - m_MaxGridDim: Prevents excessive memory for large worlds (1000×1000 = 1M cells)
         */
        typedef struct SpatialConfig
        {
            // Minimum grid cell size in world units
            // Default: 1.0f
            float m_MinCellSize;

            // Maximum grid cell size in world units
            // Default: 2.0f
            float m_MaxCellSize;

            // Maximum grid dimension in either axis
            // Default: 1000
            uint32_t m_MaxGridDim;
        } SpatialConfig;

        /**
         * @brief Single cell in spatial grid index
         *
         * Each grid cell stores indices of all NavMesh cells whose bounding
         * boxes overlap this grid cell.
         */
        typedef struct SpatialGridCell
        {
            dmArray<uint32_t> m_CellIndices; // Indices of NavMesh cells overlapping this grid cell
        } SpatialGridCell;

        /**
         * @brief Spatial grid index for O(1) cell lookup
         *
         * Partitions 2D space into uniform grid for accelerating point-in-cell queries.
         * Grid is stored as 1D array with row-major ordering: grid[y * width + x]
         *
         * Grid Bounds:
         * - m_GridMin: Bottom-left corner (minimum x/y over all cells)
         * - m_GridMax: Top-right corner (maximum x/y over all cells)
         * - Grid covers [m_GridMin, m_GridMax] with uniform cell size m_CellSize
         */
        typedef struct NavMeshSpatialIndex
        {
            SpatialGridCell* m_Grid;        // 2D array stored as 1D (row-major: grid[y*width+x])
            float            m_CellSize;    // Size of each grid cell in world units
            Vec2             m_GridMin;     // Bottom-left corner of grid
            Vec2             m_GridMax;     // Top-right corner of grid
            uint32_t         m_GridWidth;   // Grid width (number of columns)
            uint32_t         m_GridHeight;  // Grid height (number of rows)
            SpatialConfig    m_Config;      // Configuration used to build this index
            bool             m_Initialized; // Is initialized?
        } NavMeshSpatialIndex;

        /*******************************************/
        // FUNNEL ALGORITHM CONFIGURATION
        /*******************************************/

        /**
         * @brief Configuration for Simple Stupid Funnel Algorithm (SSFA)
         *
         * Controls tolerance thresholds for portal extraction and path smoothing.
         * These values handle floating-point imprecision and provide customization
         * for different world scales.
         *
         * Default Values (1 unit = 1 meter scale):
         * - m_PortalVertexTolerance: 0.002 (2mm)
         * - m_PortalCollapseThreshold: 0.1 (10% of agent diameter)
         * - m_WaypointDuplicateTolerance: 0.001 (1mm)
         *
         * Customization Guidelines:
         * - Large worlds (1 unit = 1 km): Increase all tolerances 1000×
         * - Small worlds (1 unit = 1 cm): Decrease all tolerances 100×
         */
        typedef struct FunnelConfig
        {
            // Tolerance for considering two vertices as the same point (Euclidean distance)
            // Used in portal extraction to handle floating-point imprecision when
            // detecting shared edges between cells.
            // Default: 0.002 units (2mm if unit = 1 meter)
            float m_PortalVertexTolerance;

            // Threshold for collapsing portals that are too narrow after agent radius offset
            // If portal width < agent_radius * m_PortalCollapseThreshold, collapse to midpoint.
            // This prevents agents from getting stuck in very narrow passages.
            // Default: 0.1 (portal must be > 10% of agent diameter to remain valid)
            float m_PortalCollapseThreshold;

            // Tolerance for considering two waypoints as duplicates (Euclidean distance)
            // Used to prevent redundant waypoints in the final smoothed path output.
            // Consecutive waypoints closer than this distance are merged.
            // Default: 0.001 units (1mm if unit = 1 meter)
            float m_WaypointDuplicateTolerance;
        } FunnelConfig;

        /*******************************************/
        // POLYGON NAVMESH STRUCTURE
        /*******************************************/

        /**
         * @brief Main navigation mesh container
         *
         * Contains all data required for polygon-based pathfinding:
         * - Cell array with polygon geometry and adjacency
         * - Spatial index for fast point-in-cell queries
         * - Cached polygon graph for A* pathfinding
         * - NodeId to Cell index mapping for ID-based lookups
         * - Configuration for spatial index and funnel algorithm
         *
         * Memory Ownership:
         * - All pointers are owned by the NavMesh and must be freed on shutdown
         * - m_Cells: Allocated in init(), freed in shutdown()
         * - m_SpatialIndex: Allocated in build_spatial_index(), freed in shutdown()
         * - m_PolygonGraph: Allocated lazily on first pathfinding query
         * - m_NodeToCell: Allocated in init(), freed in shutdown()
         */
        typedef struct PolygonNavMesh
        {
            Cell*                            m_Cells;         // Array of cells (pre-allocated, size = max_cells)
            uint32_t                         m_CellCount;     // Number of active cells (0 to max_cells)
            NavMeshSpatialIndex*             m_SpatialIndex;  // Spatial index for fast lookups (built on demand)
            dmHashTable<uint32_t, uint32_t>* m_NodeToCell;    // Fast node_id → cell_index lookup (O(1) hash table)
            dmArray<PolygonNode>*            m_PolygonGraph;  // Cached polygon graph (built once, reused)
            SpatialConfig                    m_SpatialConfig; // Configuration for spatial index grid sizing
        } PolygonNavMesh;

    } // namespace navmesh
} // namespace pathfinder

#endif // PATHFINDER_NAVMESH_TYPES_H
