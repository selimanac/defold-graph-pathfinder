#ifndef PATHFINDER_SMOOTH_H
#define PATHFINDER_SMOOTH_H

#include "dmarray_include.h"
#include "pathfinder_types.h"
#include <cstdint>

/**
 * @file pathfinder_smooth.h
 * @brief Path smoothing algorithms for natural and visually appealing movement
 *
 * This module provides various path smoothing algorithms to convert raw A* pathfinding
 * results (angular waypoint sequences) into smooth, natural-looking trajectories.
 *
 * Available Smoothing Methods:
 * - Catmull-Rom: Passes through all waypoints with smooth curves (C1 continuous)
 * - Bézier Quadratic: Corner-only smoothing, balanced performance (recommended)
 * - Bézier Cubic: Maximum smoothness with two control points per segment
 * - Bézier Adaptive: Configurable tightness and roundness parameters
 * - Circular Arc: Perfect circular arcs at corners (ideal for tile-based games)
 *
 * Each method has two variants:
 * - Node ID version: Takes dmArray<uint32_t> path (node IDs from pathfinding)
 * - Waypoint version: Takes dmArray<Vec2> waypoints (direct positions)
 *
 * Performance Considerations:
 * - All methods use safe_push() for automatic output array growth
 * - Sample count affects both smoothness and performance
 * - Corner-only methods (quadratic, circular arc) are faster than full-path smoothing
 * - Recommended: 8-16 samples per segment for balanced quality/performance
 *
 * Usage Pattern:
 * @code
 * dmArray<uint32_t> path;  // From find_path()
 * dmArray<Vec2> smoothed_path;
 * 
 * // Calculate required capacity (optional but recommended)
 * uint32_t capacity = pathfinder::smooth::calculate_smoothed_path_capacity(path, 8);
 * smoothed_path.SetCapacity(capacity);
 * 
 * // Apply smoothing
 * pathfinder::smooth::bezier_quadratic(path, smoothed_path, 8, 0.5f);
 * @endcode
 */

namespace pathfinder
{
    namespace smooth
    {
        /**
         * @brief Calculate required capacity for a smoothed path array
         * @param path Input path as node IDs
         * @param samples_per_segment Number of samples per segment
         * @return Conservative capacity estimate (number of Vec2 elements)
         *
         * Estimates the output array size needed for smoothing operations.
         * This is a conservative estimate that accounts for:
         * - Linear segments between waypoints
         * - Potential corner smoothing curves
         * - Safety margin for edge cases
         *
         * Formula (approximate):
         * - Corner smoothing: path_length * samples_per_segment
         * - Full path smoothing: (path_length - 1) * samples_per_segment + 1
         *
         * Use Case: Pre-allocate output array to avoid dynamic reallocation
         *
         * Example:
         * @code
         * uint32_t capacity = calculate_smoothed_path_capacity(path, 8);
         * smoothed_path.SetCapacity(capacity);
         * bezier_quadratic(path, smoothed_path, 8, 0.5f);
         * @endcode
         *
         * Note: Output may be smaller than capacity if path is mostly straight.
         * safe_push() still handles any overflow, but pre-allocation improves performance.
         */
        uint32_t calculate_smoothed_path_capacity(dmArray<uint32_t>& path, uint32_t samples_per_segment);

        /*******************************************/
        // CATMULL-ROM SPLINE SMOOTHING
        /*******************************************/

        /**
         * @brief Smooth path using Catmull-Rom splines (node ID version)
         * @param path Input path as node IDs
         * @param smoothed_path Output array (will be auto-grown if needed)
         * @param samples_per_segment Number of interpolation samples per segment
         *
         * Applies Catmull-Rom interpolation across the entire path. The resulting curve
         * passes through all original waypoints with smooth transitions between segments.
         *
         * Algorithm:
         * 1. For each segment [i] to [i+1], interpolate using waypoints [i-1], [i], [i+1], [i+2]
         * 2. Generate samples_per_segment points between [i] and [i+1]
         * 3. Uses duplicate endpoints for boundary conditions (first/last waypoints)
         *
         * Properties:
         * - C1 continuous (smooth velocity)
         * - Passes through all waypoints exactly
         * - Local control (moving one waypoint affects 2 segments on each side)
         * - No configurable tension (fixed at 0.5)
         *
         * Use Cases:
         * - Patrol routes requiring exact waypoint passage
         * - Railway/roller coaster tracks
         * - Scripted cinematics with precise positioning
         *
         * Performance: O(n * samples_per_segment) where n = path length
         * Memory: Output size ≈ (path_length - 1) * samples_per_segment + 1
         *
         * Example Output Length:
         * - 5 waypoints, 8 samples: ~33 smoothed points
         * - 10 waypoints, 16 samples: ~145 smoothed points
         */
        void catmull_rom(dmArray<uint32_t>& path, dmArray<Vec2>& smoothed_path, uint32_t samples_per_segment);

        /**
         * @brief Smooth waypoints using Catmull-Rom splines (waypoint version)
         * @param waypoints Input waypoints as Vec2 positions
         * @param smoothed_path Output array (will be auto-grown if needed)
         * @param samples_per_segment Number of interpolation samples per segment
         *
         * Same as catmull_rom() but operates on Vec2 positions directly.
         * Useful for projected paths or custom waypoint sequences.
         *
         * See catmull_rom() for detailed documentation.
         */
        void catmull_rom_waypoints(dmArray<Vec2>& waypoints, dmArray<Vec2>& smoothed_path, uint32_t samples_per_segment);

        /*******************************************/
        // QUADRATIC BÉZIER SMOOTHING (RECOMMENDED)
        /*******************************************/

        /**
         * @brief Smooth path using quadratic Bézier curves - corner-only smoothing (node ID version)
         * @param path Input path as node IDs
         * @param smoothed_path Output array (will be auto-grown if needed)
         * @param samples_per_segment Number of interpolation samples per curve (recommended: 8-16)
         * @param curve_radius Controls curve tightness [0.0, 1.0] (0.0 = minimal, 0.5 = default, 1.0 = maximum)
         *
         * **RECOMMENDED METHOD** - Best balance of performance, smoothness, and path accuracy.
         *
         * Applies quadratic Bézier smoothing only at detected corners, leaving straight
         * segments unmodified. This provides natural-looking movement while staying close
         * to the original path.
         *
         * Algorithm:
         * 1. Detect corners using is_corner() with dynamic angle threshold
         * 2. For each corner: smooth with quadratic Bézier curve
         * 3. For straight segments: use linear interpolation (or skip samples)
         * 4. Control point = corner waypoint position
         *
         * Corner Detection (dynamic based on curve_radius):
         * - Angle threshold = 179° - (curve_radius * 15°)
         * - curve_radius = 0.0: 179° threshold (only 1° turns smoothed)
         * - curve_radius = 0.5: 171.5° threshold (8.5° turns smoothed)
         * - curve_radius = 1.0: 164° threshold (16° turns smoothed)
         *
         * Smoothing Fraction (how much to smooth each corner):
         * - corner_smooth_fraction = 0.1 + (curve_radius * 0.4)
         * - curve_radius = 0.0: 10% of segment smoothed
         * - curve_radius = 0.5: 30% of segment smoothed
         * - curve_radius = 1.0: 50% of segment smoothed (maximum)
         *
         * Properties:
         * - Corner-only smoothing (efficient, minimal path deviation)
         * - Configurable aggressiveness via curve_radius parameter
         * - Preserves straight segments exactly
         * - Natural-looking rounded corners
         *
         * Use Cases:
         * - General-purpose character/vehicle movement
         * - Real-time games with many agents
         * - Grid-based pathfinding results
         * - Any scenario where slight path deviation is acceptable
         *
         * Performance: O(n * samples_per_segment) where n = number of corners (not path length)
         * Memory: Output size ≈ path_length + (corner_count * samples_per_segment)
         *
         * Example Output Length:
         * - 10 waypoints, 3 corners, 8 samples: ~31 points (10 + 3*8 - 3)
         * - Straight path (0 corners): ~10 points (unchanged)
         *
         * Tuning Guide:
         * - Increase samples_per_segment for smoother corners (8-32 typical)
         * - Increase curve_radius for more aggressive smoothing (0.3-0.7 typical)
         * - For tight spaces: curve_radius = 0.2-0.3
         * - For open areas: curve_radius = 0.6-0.8
         *
         * @code
         * // Conservative smoothing (tight spaces)
         * bezier_quadratic(path, smoothed, 8, 0.3f);
         * 
         * // Balanced smoothing (recommended)
         * bezier_quadratic(path, smoothed, 12, 0.5f);
         * 
         * // Aggressive smoothing (open areas)
         * bezier_quadratic(path, smoothed, 16, 0.7f);
         * @endcode
         */
        void bezier_quadratic(dmArray<uint32_t>& path, dmArray<Vec2>& smoothed_path, uint32_t samples_per_segment, float curve_radius = 0.5f);

        /**
         * @brief Smooth waypoints using quadratic Bézier curves - corner-only smoothing (waypoint version)
         * @param waypoints Input waypoints as Vec2 positions (e.g., start, entry_point, path nodes)
         * @param smoothed_path Output array (will be auto-grown if needed)
         * @param samples_per_segment Number of interpolation samples per curve (recommended: 8-16)
         * @param curve_radius Controls curve tightness [0.0, 1.0] (0.0 = minimal, 0.5 = default, 1.0 = maximum)
         * @param skip_second_waypoint_corner If true, prevents waypoint[1] from being smoothed as a corner
         *
         * Same as bezier_quadratic() but operates on Vec2 positions directly.
         * 
         * Special Parameter:
         * - skip_second_waypoint_corner: Useful for projected paths where waypoint[1] is
         *   the entry point (projection onto graph edge). Prevents smoothing at the entry
         *   point to ensure the agent enters the graph precisely.
         *
         * Use Cases:
         * - Projected pathfinding (find_path_projected results)
         * - Custom waypoint sequences
         * - Mixed node/position paths
         *
         * Example (Projected Path):
         * @code
         * dmArray<Vec2> waypoints;
         * waypoints.Push(agent_position);      // Current position
         * waypoints.Push(entry_point);         // Projection point (don't smooth this!)
         * for (uint32_t node_id : path) {
         *     waypoints.Push(get_node_position(node_id));
         * }
         * bezier_quadratic_waypoints(waypoints, smoothed, 8, 0.5f, true); // skip_second = true
         * @endcode
         */
        void bezier_quadratic_waypoints(dmArray<Vec2>& waypoints, dmArray<Vec2>& smoothed_path, uint32_t samples_per_segment, float curve_radius = 0.5f, bool skip_second_waypoint_corner = false);

        /*******************************************/
        // CUBIC BÉZIER SMOOTHING
        /*******************************************/

        /**
         * @brief Smooth path using cubic Bézier curves (node ID version)
         * @param path Input path as node IDs
         * @param smoothed_path Output array (will be auto-grown if needed)
         * @param samples_per_segment Number of interpolation samples per segment
         * @param control_point_offset Control point distance multiplier
         *
         * Applies cubic Bézier interpolation across the entire path using two control
         * points per segment for maximum smoothness.
         *
         * Properties:
         * - Very smooth curves (smoother than quadratic)
         * - Does not pass through original waypoints (approximating)
         * - More deviation from original path than quadratic
         * - Slower than quadratic or corner-only methods
         *
         * Use Cases:
         * - Cinematic camera paths
         * - High-quality showcase sequences
         * - Artistic control over path aesthetics
         *
         * Performance: O(n * samples_per_segment) where n = path length
         * Memory: Output size ≈ (path_length - 1) * samples_per_segment + 1
         */
        void bezier_cubic(dmArray<uint32_t>& path, dmArray<Vec2>& smoothed_path, uint32_t samples_per_segment, float control_point_offset);

        /**
         * @brief Smooth waypoints using cubic Bézier curves (waypoint version)
         * @param waypoints Input waypoints as Vec2 positions
         * @param smoothed_path Output array (will be auto-grown if needed)
         * @param samples_per_segment Number of interpolation samples per segment
         * @param control_point_offset Control point distance multiplier
         *
         * Same as bezier_cubic() but operates on Vec2 positions directly.
         */
        void bezier_cubic_waypoints(dmArray<Vec2>& waypoints, dmArray<Vec2>& smoothed_path, uint32_t samples_per_segment, float control_point_offset);

        /*******************************************/
        // ADAPTIVE BÉZIER SMOOTHING
        /*******************************************/

        /**
         * @brief Adaptive corner smoothing with configurable tightness and roundness (node ID version)
         * @param path Input path as node IDs
         * @param smoothed_path Output array (will be auto-grown if needed)
         * @param samples_per_segment Number of interpolation samples per corner curve
         * @param tightness Corner tightness [0.0, 1.0] (0.0 = loose curves, 1.0 = tight corners)
         * @param roundness Curve roundness [0.0, 1.0] (0.0 = flat curves, 1.0 = very round)
         * @param max_corner_dist Maximum distance from corner point for control point placement
         *
         * Advanced corner smoothing with independent control over curve shape and extent.
         * Provides maximum flexibility for fine-tuning path aesthetics.
         *
         * Parameters:
         * - tightness: Controls how close curves stay to corner points
         *   - 0.0: Curves extend far from corners (gentle, wide arcs)
         *   - 1.0: Curves stay close to corners (sharp, tight turns)
         * - roundness: Controls curve "bulge" or roundness
         *   - 0.0: Flatter curves (more linear between control points)
         *   - 1.0: Rounder curves (more pronounced bulge)
         * - max_corner_dist: Caps control point distance from corner
         *   - Prevents overly large curves in open spaces
         *   - Typical values: 20-100 world units
         *
         * Use Cases:
         * - Highly customized path aesthetics
         * - Matching specific art direction requirements
         * - Fine-tuning for different agent types (fast vs slow)
         *
         * Performance: O(n * samples_per_segment) where n = number of corners
         * Note: Slower than bezier_quadratic due to additional parameter calculations
         *
         * Tuning Examples:
         * @code
         * // Gentle, wide arcs
         * bezier_adaptive(path, smoothed, 12, 0.3f, 0.5f, 50.0f);
         * 
         * // Tight, sharp corners
         * bezier_adaptive(path, smoothed, 8, 0.8f, 0.3f, 20.0f);
         * 
         * // Very round, sweeping curves
         * bezier_adaptive(path, smoothed, 16, 0.4f, 0.9f, 80.0f);
         * @endcode
         */
        void bezier_adaptive(dmArray<uint32_t>& path,
                             dmArray<Vec2>&     smoothed_path,
                             uint32_t           samples_per_segment,
                             float              tightness = 0.5f,
                             float              roundness = 0.5f,
                             float              max_corner_dist = 50.0f);

        /**
         * @brief Adaptive corner smoothing with configurable parameters (waypoint version)
         * @param waypoints Input waypoints as Vec2 positions
         * @param smoothed_path Output array (will be auto-grown if needed)
         * @param samples_per_segment Number of interpolation samples per corner curve
         * @param tightness Corner tightness [0.0, 1.0] (0.0 = loose curves, 1.0 = tight corners)
         * @param roundness Curve roundness [0.0, 1.0] (0.0 = flat curves, 1.0 = very round)
         * @param max_corner_dist Maximum distance from corner point for control point placement
         *
         * Same as bezier_adaptive() but operates on Vec2 positions directly.
         * See bezier_adaptive() for detailed parameter documentation.
         */
        void bezier_adaptive_waypoints(dmArray<Vec2>& waypoints,
                                       dmArray<Vec2>& smoothed_path,
                                       uint32_t       samples_per_segment,
                                       float          tightness = 0.5f,
                                       float          roundness = 0.5f,
                                       float          max_corner_dist = 50.0f);

        /*******************************************/
        // CIRCULAR ARC SMOOTHING
        /*******************************************/

        /**
         * @brief Smooth path corners using perfect circular arcs (node ID version)
         * @param path Input path as node IDs
         * @param smoothed_path Output array (will be auto-grown if needed)
         * @param samples_per_segment Number of interpolation samples per arc (recommended: 8-16)
         * @param arc_radius Radius of circular arcs in world units (determines corner smoothness)
         *
         * Applies perfect circular arc smoothing at corners. Creates mathematically precise
         * circular curves that blend seamlessly with straight segments.
         *
         * Algorithm:
         * 1. Detect corners (3 consecutive non-collinear waypoints)
         * 2. Calculate tangent lines from corner waypoint to adjacent segments
         * 3. Place arc endpoints on tangent lines at distance determined by arc_radius
         * 4. Generate circular arc between endpoints (center determined by perpendicular bisectors)
         * 5. Arc radius is clamped to half the shorter adjacent segment (prevents overlaps)
         *
         * Properties:
         * - Perfect circular arcs (constant curvature)
         * - Tangent continuous (C1) with straight segments
         * - Predictable path deviation (controlled by arc_radius)
         * - Ideal for tile-based games and grid movement
         *
         * Arc Radius Considerations:
         * - Small radius (10-20): Tight corners, stays close to original path
         * - Medium radius (30-50): Balanced smoothness and accuracy
         * - Large radius (60-100): Very smooth, sweeping curves
         * - Auto-clamped: Never exceeds half the shorter adjacent segment length
         *
         * Use Cases:
         * - Tile-based pathfinding (grid movement)
         * - Railroad/road/track systems
         * - Vehicle movement requiring constant turn radius
         * - Scenarios where circular turns are physically natural
         *
         * Performance: O(n * samples_per_segment) where n = number of corners
         * Memory: Output size ≈ path_length + (corner_count * samples_per_segment)
         *
         * Example Output Length:
         * - 10 waypoints, 4 corners, 12 samples: ~58 points (10 + 4*12)
         * - Straight path (0 corners): ~10 points (unchanged)
         *
         * Comparison with Bézier:
         * - Circular arc: Constant curvature, predictable radius
         * - Bézier quadratic: Variable curvature, more natural for organic movement
         * - Circular arc is preferred for mechanical/vehicle movement
         * - Bézier is preferred for character/organic movement
         *
         * Tuning Examples:
         * @code
         * // Tight grid turns (tile-based game)
         * circular_arc(path, smoothed, 8, 15.0f);
         * 
         * // Balanced smoothness (general purpose)
         * circular_arc(path, smoothed, 12, 30.0f);
         * 
         * // Very smooth railroad curves
         * circular_arc(path, smoothed, 16, 60.0f);
         * @endcode
         *
         * Note: Arc radius is automatically clamped per corner to prevent overlapping arcs.
         * If two corners are very close together, effective radius may be smaller than specified.
         */
        void circular_arc(dmArray<uint32_t>& path, dmArray<Vec2>& smoothed_path, uint32_t samples_per_segment, float arc_radius);

        /**
         * @brief Smooth waypoints using perfect circular arcs (waypoint version)
         * @param waypoints Input waypoints as Vec2 positions
         * @param smoothed_path Output array (will be auto-grown if needed)
         * @param samples_per_segment Number of interpolation samples per arc (recommended: 8-16)
         * @param arc_radius Radius of circular arcs in world units (determines corner smoothness)
         * @param skip_second_waypoint_corner If true, prevents waypoint[1] from being smoothed as a corner
         *
         * Same as circular_arc() but operates on Vec2 positions directly.
         *
         * Special Parameter:
         * - skip_second_waypoint_corner: Useful for projected paths where waypoint[1] is
         *   the entry point. Prevents arc smoothing at the entry point to ensure precise
         *   graph entry.
         *
         * Use Cases:
         * - Projected pathfinding results
         * - Custom waypoint sequences
         * - Mixed node/position paths
         *
         * Example (Projected Path):
         * @code
         * dmArray<Vec2> waypoints;
         * waypoints.Push(agent_position);  // Current position
         * waypoints.Push(entry_point);     // Projection point (don't smooth!)
         * for (uint32_t node_id : path) {
         *     waypoints.Push(get_node_position(node_id));
         * }
         * circular_arc_waypoints(waypoints, smoothed, 12, 25.0f, true); // skip_second = true
         * @endcode
         *
         * See circular_arc() for detailed arc radius and performance documentation.
         */
        void circular_arc_waypoints(dmArray<Vec2>& waypoints, dmArray<Vec2>& smoothed_path, uint32_t samples_per_segment, float arc_radius, bool skip_second_waypoint_corner = false);

        /**
         * @brief Generate a circular arc between three waypoints with a specified arc angle
         * @param p0 Start waypoint (arc begins tangent to segment p0→p1)
         * @param p1 Corner waypoint (arc is centered around this point conceptually)
         * @param p2 End waypoint (arc ends tangent to segment p1→p2)
         * @param smoothed_path Output array for arc points
         * @param samples Number of points to generate on the arc
         * @param arc_angle_degrees Desired arc angle in degrees (e.g., 90 for quarter circle, 180 for semicircle)
         * @param arc_radius Radius of the circular arc (if 0, automatically calculated to fit the three points)
         * @return true if arc was successfully generated, false if waypoints are invalid
         *
         * Creates a circular arc piece with a specific turning angle. This is particularly
         * useful for creating railroad tracks, road segments, or other infrastructure where
         * specific turn angles are required.
         *
         * Algorithm:
         * 1. Calculate turn angle at p1 from vectors (p0→p1) and (p1→p2)
         * 2. If arc_radius = 0: Calculate radius to create perfect arc through all 3 points
         * 3. If arc_radius > 0: Use specified radius (may not pass through p1 exactly)
         * 4. Determine arc center using perpendicular from p1
         * 5. Generate 'samples' points along the arc
         *
         * Arc Angle Examples:
         * - 30°: Gentle curve (slight direction change)
         * - 45°: Moderate curve
         * - 90°: Quarter circle (right-angle turn)
         * - 135°: Sharp curve
         * - 180°: Semicircle (U-turn)
         *
         * Auto-radius Calculation (arc_radius = 0):
         * - Determines radius that creates an arc passing through or near all 3 waypoints
         * - Useful when exact geometry is more important than specific radius
         *
         * Fixed Radius (arc_radius > 0):
         * - Uses specified radius regardless of waypoint geometry
         * - Arc may not pass through p1 exactly
         * - Useful for standardized track pieces (e.g., all 90° turns use 50-unit radius)
         *
         * Use Cases:
         * - Railroad/road piece generation (standardized curves)
         * - Level editor arc primitives
         * - Procedural track/path generation
         * - Tower defense path segments
         *
         * Validation:
         * - Returns false if waypoints are collinear (no turn, can't create arc)
         * - Returns false if waypoints are too close together (degenerate case)
         * - Returns true if arc successfully generated
         *
         * Example (Railway Track Pieces):
         * @code
         * // Generate a 90° turn piece with 50-unit radius
         * Vec2 start = {0, 0};
         * Vec2 corner = {100, 0};
         * Vec2 end = {100, 100};
         * dmArray<Vec2> arc_points;
         * bool success = circular_arc_corner(start, corner, end, arc_points, 16, 90.0f, 50.0f);
         * 
         * if (success) {
         *     // Use arc_points for rendering/collision
         * }
         * @endcode
         *
         * Example (Auto-fitted Arc):
         * @code
         * // Let the algorithm determine optimal radius
         * bool success = circular_arc_corner(p0, p1, p2, arc_points, 12, 45.0f, 0.0f);
         * @endcode
         *
         * Performance: O(samples)
         * Memory: Adds 'samples' Vec2 points to smoothed_path
         */
        bool circular_arc_corner(Vec2 p0, Vec2 p1, Vec2 p2, dmArray<Vec2>& smoothed_path, uint32_t samples, float arc_angle_degrees, float arc_radius = 0.0f);

    } // namespace smooth
} // namespace pathfinder

#endif