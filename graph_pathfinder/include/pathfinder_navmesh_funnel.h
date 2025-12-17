/**
 * @file pathfinder_navmesh_funnel.h
 * @brief Funnel Algorithm for NavMesh Path Smoothing (String Pulling)
 *
 * Implementation of the Simple Stupid Funnel Algorithm (SSFA) for finding the
 * shortest path through a polygon corridor in a navigation mesh.
 *
 * ## Algorithm Overview
 *
 * The Funnel Algorithm converts a corridor of adjacent polygons into an optimal
 * shortest path by:
 * 1. Extracting portals (shared edges) between consecutive polygon pairs
 * 2. Maintaining a "funnel" (triangular region) defined by:
 *    - An apex point (current path vertex)
 *    - Left and right edges (boundaries of possible paths)
 * 3. Iteratively narrowing the funnel as new portals are processed
 * 4. Emitting waypoints when the funnel "closes" (one edge crosses the other)
 *
 * ## Key Features
 *
 * - **Optimal Paths**: Produces true shortest paths through polygon corridors
 * - **Agent Radius Support**: Runtime collision avoidance via portal offsetting
 * - **Robust Edge Cases**: Handles degenerate portals, narrow passages, single-cell paths
 * - **Duplicate Prevention**: Filters redundant waypoints for cleaner output
 * - **Performance**: O(n) complexity for n portals with hash table optimization
 *
 * ## Comparison with Detour/Recast
 *
 * This implementation is based on the same algorithm as Detour (Recast Navigation),
 * with enhancements:
 * - Runtime agent radius offsetting (Detour uses pre-shrunk navmeshes)
 * - Explicit duplicate waypoint prevention
 * - Graceful degradation for narrow portals (collapse to midpoint)
 * - Configurable tolerance for floating-point comparisons
 *
 * ## References
 *
 * - Demyen & Buro (2006): "Efficient Triangulation-Based Pathfinding"
 * - Detour NavMeshQuery: https://github.com/recastnavigation/recastnavigation
 * - Research paper: https://idm-lab.org/bib/abstracts/papers/socs20c.pdf
 *
 * ## Usage Example
 *
 * @code
 * // After finding a cell path with A* on the navmesh graph:
 * dmArray<uint32_t> cell_path;  // From A* pathfinding
 * dmArray<Vec2> smooth_path;
 *
 * uint32_t waypoint_count = pathfinder::navmesh::funnel::apply_funnel(
 *     navmesh,
 *     cell_path,
 *     start_position,
 *     goal_position,
 *     &smooth_path,
 *     agent_radius  // e.g., 5.0f for 5-unit radius agent
 * );
 *
 * // smooth_path now contains optimal waypoints
 * @endcode
 *
 * @note The algorithm requires a valid polygon corridor (cell_path) where consecutive
 *       cells share edges. Disconnected cells will produce fallback portals.
 *
 * Source: Extracted and refactored from raylib_navmesh_buffer.cpp lines 709-1118
 */

#ifndef PATHFINDER_NAVMESH_FUNNEL_H
#define PATHFINDER_NAVMESH_FUNNEL_H

#include "dmarray_include.h"
#include "pathfinder_navmesh_types.h"
#include "pathfinder_types.h"
#include <stdint.h>

namespace pathfinder
{
    namespace navmesh
    {
        namespace funnel
        {
            // ============================================================================
            // FUNNEL ALGORITHM
            // ============================================================================

            /**
             * @brief Apply Simple Stupid Funnel Algorithm to smooth a polygon corridor path
             *
             * Converts a corridor of adjacent navmesh cells into an optimal shortest path
             * by extracting portals (shared edges) and maintaining a narrowing funnel.
             * This is the core path smoothing operation for navmesh pathfinding.
             *
             * ## Algorithm Steps
             *
             * 1. **Build Portal List**:
             *    - Extract shared edges between consecutive cell pairs
             *    - Add degenerate start/goal portals (single points)
             *    - Offset portals inward by agent_radius (if > 0)
             *
             * 2. **Initialize Funnel**:
             *    - Apex = start_pos
             *    - Left edge = start_pos
             *    - Right edge = start_pos
             *
             * 3. **Process Portals** (for each portal i):
             *    - Try to narrow right edge:
             *      - If new right is inside funnel → update right edge
             *      - If new right crosses left edge → emit left vertex, restart from left
             *    - Try to narrow left edge:
             *      - If new left is inside funnel → update left edge
             *      - If new left crosses right edge → emit right vertex, restart from right
             *
             * 4. **Finalize**: Append goal_pos as final waypoint
             *
             * ## Edge Cases Handled
             *
             * - **Single cell**: Returns direct path [start, goal]
             * - **Degenerate portals**: Uses cell center midpoints as fallback
             * - **Narrow portals**: Collapses to midpoint if < agent_radius * 0.2
             * - **Duplicate waypoints**: Filtered via 0.001-unit tolerance
             * - **Missing cells**: Uses fallback degenerate portals at start_pos
             *
             * ## Performance
             *
             * - Time: O(n) for n portals (may rescan up to n times in worst case)
             * - Space: O(n) for portal storage
             * - Cell lookup: O(1) via hash table (or O(m) fallback for m cells)
             *
             * ## Agent Radius Behavior
             *
             * When agent_radius > 0:
             * - Portals are offset inward (shrunk) by agent_radius
             * - This provides collision avoidance at path planning time
             * - Differs from Detour (which uses pre-shrunk navmeshes)
             * - If portal becomes too narrow (< agent_radius * 0.2), collapses to midpoint
             *
             * @param navmesh Polygon navmesh containing cells and topology
             * @param cell_path Array of cell IDs forming the corridor (from A* on navmesh graph)
             * @param start_pos Starting position (typically inside first cell)
             * @param goal_pos Goal position (typically inside last cell)
             * @param out_smooth_path Output array for smoothed waypoints (cleared on entry)
             * @param agent_radius Radius of agent for collision avoidance (0 = no offset)
             *
             * @return Number of waypoints in smooth_path (>= 2 if successful, 0 on failure)
             *
             * @note out_smooth_path is cleared at the start. Ensure it has sufficient capacity
             *       (typically cell_path.Size() + 2 is safe upper bound).
             *
             * @warning cell_path must contain valid cell IDs that exist in navmesh. Invalid
             *          IDs will produce fallback degenerate portals, potentially causing
             *          suboptimal paths.
             *
             * ## Example
             *
             * @code
             * PolygonNavMesh navmesh;
             * dmArray<uint32_t> cell_path;
             * // ... A* pathfinding populates cell_path ...
             *
             * dmArray<Vec2> smooth_path;
             * smooth_path.SetCapacity(cell_path.Size() + 2);
             *
             * uint32_t waypoints = pathfinder::navmesh::funnel::apply_funnel(
             *     &navmesh, cell_path, {10, 10}, {500, 500}, &smooth_path, 5.0f
             * );
             *
             * if (waypoints >= 2) {
             *     // Use smooth_path for agent movement
             *     for (uint32_t i = 0; i < waypoints; i++) {
             *         printf("Waypoint %u: (%.2f, %.2f)\n", i,
             *                smooth_path[i].x, smooth_path[i].y);
             *     }
             * }
             * @endcode
             *
             * @see get_portal_between_cells For portal extraction logic
             * @see offset_portal For agent radius collision avoidance
             */
            uint32_t apply_funnel(PolygonNavMesh*     navmesh,
                                  dmArray<uint32_t>&  cell_path,
                                  Vec2                start_pos,
                                  Vec2                goal_pos,
                                  dmArray<Vec2>*      out_smooth_path,
                                  float               agent_radius,
                                  const FunnelConfig* config);

            // ============================================================================
            // PORTAL UTILITIES
            // ============================================================================

            /**
             * @brief Extract the shared edge (portal) between two adjacent navmesh cells
             *
             * Finds the two vertices that are shared between cells 'a' and 'b', forming
             * the portal through which paths can pass from one cell to the other. The
             * portal vertices are ordered as "left" and "right" relative to the direction
             * from cell 'a' to cell 'b'.
             *
             * ## Algorithm
             *
             * 1. **Find Shared Vertices**: O(N×M) brute-force search with tolerance
             *    - Compare each vertex of 'a' with each vertex of 'b'
             *    - Vertices are considered shared if distance² < tolerance²
             *    - Stop after finding 2 shared vertices
             *
             * 2. **Determine Left/Right Ordering**:
             *    - Compute direction vector: dir = center(b) - center(a)
             *    - Compute cross product: cross = (e1 - e0) × dir
             *    - If cross > 0: left = e0, right = e1
             *    - If cross <= 0: left = e1, right = e0
             *
             * ## Tolerance Handling
             *
             * - Tolerance: 0.002 units (squared = 0.000004)
             * - Handles floating-point imprecision in vertex positions
             * - Sufficient for typical game scales (1 unit = 1 meter)
             * - May need adjustment for very large or very small worlds
             *
             * ## Edge Cases
             *
             * - **No shared vertices**: Returns invalid portal (m_Valid = false)
             * - **< 2 shared vertices**: Returns invalid portal
             * - **Degenerate cells** (coincident vertices): May find same vertex twice
             *
             * @param a First navmesh cell (polygon)
             * @param b Second navmesh cell (polygon), should be adjacent to 'a'
             *
             * @return Portal structure containing:
             *         - m_Left: Left vertex of the portal (from a's perspective toward b)
             *         - m_Right: Right vertex of the portal
             *         - m_Valid: true if exactly 2 shared vertices found, false otherwise
             *
             * @note The returned portal is "unordered" in the sense that left/right
             *       is relative to the a→b direction, not absolute world coordinates.
             *       This is correct for the funnel algorithm, which processes portals
             *       in sequence along the path.
             *
             * @warning O(N×M) complexity where N, M are vertex counts. For performance-
             *          critical applications with large polygons, consider pre-computing
             *          portal edges during navmesh build.
             *
             * ## Performance Optimization Notes
             *
             * Current: O(N×M) brute force (worst case 64 comparisons for two 8-gons)
             * Alternatives:
             * - Pre-compute portals during navmesh build: O(1) lookup
             * - Hash vertices by position: O(N + M) with hash table
             * - For triangular navmeshes: Always 2 shared vertices, could specialize
             *
             * @see offset_portal To adjust portal for agent radius collision avoidance
             */
            Portal get_portal_between_cells(Cell* a, Cell* b, const FunnelConfig* config);

            /**
             * @brief Offset portal vertices inward for agent radius collision avoidance
             *
             * Shrinks a portal by moving both endpoints toward each other along the
             * portal edge. This ensures that paths passing through the portal maintain
             * a minimum clearance from polygon edges equal to agent_radius.
             *
             * ## Algorithm
             *
             * 1. **Compute Edge Vector**: edge = right - left
             * 2. **Normalize**: edge_dir = edge / |edge|
             * 3. **Offset Endpoints**:
             *    - new_left = left + edge_dir * agent_radius
             *    - new_right = right - edge_dir * agent_radius
             * 4. **Validate Width**: If new_width < agent_radius * 0.1, collapse to midpoint
             *
             * ## Collapse Threshold
             *
             * Portals narrower than 10% of agent diameter (agent_radius * 0.2) are
             * collapsed to their midpoint. This prevents:
             * - Invalid portals where left crosses right
             * - Numerical instability in the funnel algorithm
             * - Paths that attempt to squeeze through impossibly narrow gaps
             *
             * ## Comparison with Detour/Recast
             *
             * **Detour Approach**: Pre-shrinks navmesh during build (erosion)
             * - Pros: Faster at runtime (no offsetting needed)
             * - Cons: Requires separate navmesh per agent size
             *
             * **This Implementation**: Runtime portal offsetting
             * - Pros: Single navmesh supports multiple agent sizes
             * - Cons: Small runtime cost per portal
             *
             * ## Edge Cases
             *
             * - **agent_radius = 0**: Returns portal unmodified (no offsetting)
             * - **Degenerate portal** (left == right): Returns unmodified (no edge to offset)
             * - **Portal too narrow**: Collapses to midpoint (see threshold above)
             * - **Very short edge** (< 0.001): Returns unmodified
             *
             * @param portal The portal to offset (shared edge between two cells)
             * @param agent_radius Radius of the agent in world units (e.g., 5.0 for 5-unit radius)
             *
             * @return Offset portal with:
             *         - m_Left moved inward by agent_radius
             *         - m_Right moved inward by agent_radius
             *         - m_Valid preserved from input portal
             *         - Collapsed to midpoint if resulting width < agent_radius * 0.2
             *
             * @note The 0.1× threshold (agent_radius * 0.1 per side, 0.2 total) is a
             *       heuristic chosen for stability. Very narrow portals likely indicate
             *       areas the agent cannot traverse anyway.
             *
             * @warning If many portals collapse to midpoints, consider:
             *          1. Reducing agent_radius
             *          2. Regenerating navmesh with larger polygons
             *          3. Using a pre-shrunk navmesh (Detour-style)
             *
             * ## Example
             *
             * @code
             * Portal p;
             * p.m_Left = {0, 0};
             * p.m_Right = {100, 0};  // 100-unit wide portal
             * p.m_Valid = true;
             *
             * Portal offset = offset_portal(p, 5.0f);
             * // offset.m_Left = {5, 0}
             * // offset.m_Right = {95, 0}
             * // Effective width: 90 units (agent has 5-unit clearance on each side)
             *
             * Portal narrow;
             * narrow.m_Left = {0, 0};
             * narrow.m_Right = {1, 0};  // 1-unit wide portal
             *
             * Portal collapsed = offset_portal(narrow, 5.0f);
             * // collapsed.m_Left = {0.5, 0} (midpoint)
             * // collapsed.m_Right = {0.5, 0} (midpoint)
             * // Portal too narrow for 5-unit agent → collapsed
             * @endcode
             */
            Portal offset_portal(Portal portal, float agent_radius, const FunnelConfig* config);

        } // namespace funnel
    } // namespace navmesh
} // namespace pathfinder

#endif // PATHFINDER_NAVMESH_FUNNEL_H
