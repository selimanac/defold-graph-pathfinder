#ifndef PATHFINDER_CONST_H
#define PATHFINDER_CONST_H

#include <cstdint>

/**
 * @file pathfinder_constants.h
 * @brief Core constants, enumerations, and configuration values for the pathfinding system
 *
 * This header defines fundamental constants, error codes, and configuration enums used
 * throughout the pathfinding engine. All values are compile-time constants to ensure
 * optimal performance and type safety.
 *
 * Key Components:
 * - INVALID_ID constant for invalid IDs and error returns
 * - Mathematical constants (EPSILON, PI)
 * - PathStatus enum for detailed error reporting
 * - PathSmoothStyle enum for path smoothing algorithms
 * - Smoothing configuration constants for corner detection and curve parameters
 *
 * Usage Pattern:
 * - Check function returns against PathStatus values for error handling
 * - Use INVALID_ID constant (UINT32_MAX) to detect invalid node/edge IDs
 * - Select PathSmoothStyle based on desired movement characteristics
 * - Adjust smooth_constants to tune corner smoothing behavior
 *
 * Thread Safety: All constants are read-only and thread-safe
 */

namespace pathfinder
{
    /**
     * @brief Invalid ID constant for nodes and edges
     *
     * Used to indicate:
     * - Invalid/uninitialized node ID
     * - Failed node creation (add_node returns INVALID_ID)
     * - Empty heap pop result
     * - Projection failure in find_path_projected
     *
     * Value: UINT32_MAX (4,294,967,295)
     * Rationale: Maximum uint32_t value, unlikely to be a valid ID
     *
     * Note: Renamed from ERROR to avoid conflicts with Windows macro
     */
    static const uint32_t INVALID_ID = UINT32_MAX;

    /**
     * @brief Floating point comparison epsilon for near-zero checks
     *
     * Used for:
     * - Vector length comparisons (avoid division by zero)
     * - Position equality checks (node movement detection)
     * - Angle calculations (detect degenerate cases)
     *
     * Value: 0.0001f (1e-4)
     * Smaller values increase precision but risk false negatives
     */
    static const float EPSILON = 0.0001f;

    /**
     * @brief Mathematical constant π (pi)
     *
     * Used for:
     * - Angle conversions (degrees ↔ radians)
     * - Circular arc calculations
     * - Trigonometric functions
     *
     * Value: 3.14159265358979323846f
     * Precision: Single-precision float (7 significant digits)
     */
    static const float MATH_PI = 3.14159265358979323846f;

    /**
     * @brief Path smoothing algorithm selection
     *
     * Determines which smoothing algorithm to apply to a raw pathfinding result.
     * Each style has different characteristics for curve quality, performance,
     * and path deviation from original waypoints.
     *
     * Performance Order (fastest to slowest):
     * NONE < BEZIER_QUADRATIC < CATMULL_ROM < CIRCULAR_ARC < BEZIER_CUBIC < BEZIER_ADAPTIVE
     *
     * Path Accuracy (closest to original waypoints to most deviated):
     * NONE < CATMULL_ROM < CIRCULAR_ARC < BEZIER_QUADRATIC < BEZIER_CUBIC < BEZIER_ADAPTIVE
     */
    enum PathSmoothStyle
    {
        NONE = 0,             ///< No smoothing - use raw waypoints (fastest, angular paths)
        CATMULL_ROM = 1,      ///< Passes through all waypoints with smooth curves (moderate performance)
        BEZIER_CUBIC = 2,     ///< Very smooth approximating curves, two control points (slowest, most deviation)
        BEZIER_QUADRATIC = 3, ///< Corner-only smoothing, one control point (recommended, good balance)
        BEZIER_ADAPTIVE = 4,  ///< Adaptive corner smoothing with configurable tightness (slow, highly customizable)
        CIRCULAR_ARC = 5      ///< Perfect circular arcs at corners (best for tile-based/grid movement)
    };

    /**
     * @brief Status codes for pathfinding and graph operations
     *
     * All functions that can fail return a PathStatus value (either directly
     * or via output parameter). Successful operations return SUCCESS (0).
     * Negative values indicate specific error conditions.
     *
     * Error Handling Pattern:
     * @code
     * PathStatus status;
     * uint32_t node_id = pathfinder::path::add_node(pos, &status);
     * if (status != SUCCESS) {
     *     // Handle error based on status code
     * }
     * @endcode
     */
    enum PathStatus
    {
        SUCCESS = 0, ///< Operation completed successfully

        // Pathfinding Errors (path not found or unreachable)
        ERROR_NO_PATH = -1,               ///< No valid path exists between start and goal nodes
        ERROR_START_GOAL_NODE_SAME = -12, ///< Start node ID and Goal node ID are same

        // Node Validation Errors
        ERROR_START_NODE_INVALID = -2, ///< Start node ID is invalid, inactive, or out of bounds
        ERROR_GOAL_NODE_INVALID = -3,  ///< Goal node ID is invalid, inactive, or out of bounds

        // Capacity Errors (system limits reached)
        ERROR_NODE_FULL = -4,     ///< Maximum node capacity reached, cannot add more nodes
        ERROR_EDGE_FULL = -5,     ///< Node's edge capacity full, cannot add more edges
        ERROR_HEAP_FULL = -6,     ///< Heap pool exhausted during pathfinding (increase pool_block_size)
        ERROR_PATH_TOO_LONG = -7, ///< Path exceeds max_path length limit (currently unused)

        // Graph Consistency Errors
        ERROR_GRAPH_CHANGED = -8,            ///< Graph was modified during pathfinding (triggers automatic retry)
        ERROR_GRAPH_CHANGED_TOO_OFTEN = -11, ///< Graph changed too many times during pathfinding (>3 retries)

        // Projected Pathfinding Errors
        ERROR_NO_PROJECTION = -9,       ///< Cannot project point onto graph (no edges exist)
        ERROR_VIRTUAL_NODE_FAILED = -10 ///< Failed to create/connect virtual node for projection
    };

    /**
     * @brief Configuration constants for BEZIER_QUADRATIC path smoothing
     *
     * These constants control the behavior of the quadratic Bézier corner smoothing
     * algorithm (bezier_quadratic_waypoints). They determine:
     * - How much of each segment to smooth (corner_smooth_fraction)
     * - Which corners need smoothing (corner_angle_threshold)
     *
     * The curve_radius parameter (0.0 to 1.0) interpolates between minimum and
     * maximum smoothing aggressiveness:
     * - curve_radius = 0.0: Conservative smoothing, only very sharp corners
     * - curve_radius = 0.5: Balanced smoothing (recommended)
     * - curve_radius = 1.0: Aggressive smoothing, gentle curves at most corners
     *
     * Tuning Guide:
     * - Increase MIN_CORNER_SMOOTH_FRACTION for rounder minimum curves
     * - Increase CORNER_SMOOTH_RANGE for stronger curve_radius effect
     * - Decrease MAX_CORNER_ANGLE_THRESHOLD to smooth only sharper corners
     * - Increase CORNER_ANGLE_RANGE for stronger curve_radius corner detection
     */
    namespace smooth_constants
    {
        /**
         * @brief Minimum corner smoothing fraction (percentage of segment length)
         *
         * When curve_radius = 0.0, corners are smoothed by 10% of the adjacent
         * segment lengths. This ensures at least some smoothing even with minimal
         * curve_radius, preventing completely angular paths.
         *
         * Value: 0.1f (10% of segment length)
         * Range: 0.0f to 0.5f (cannot exceed half segment length)
         * Effect: Higher values create rounder minimum curves
         */
        static const float MIN_CORNER_SMOOTH_FRACTION = 0.1f;

        /**
         * @brief Additional corner smoothing range (added with curve_radius)
         *
         * The smoothing fraction increases linearly from MIN_CORNER_SMOOTH_FRACTION
         * to (MIN_CORNER_SMOOTH_FRACTION + CORNER_SMOOTH_RANGE) as curve_radius
         * goes from 0.0 to 1.0.
         *
         * Formula: corner_smooth_fraction = MIN_CORNER_SMOOTH_FRACTION + (curve_radius * CORNER_SMOOTH_RANGE)
         *
         * Value: 0.4f
         * Result range: 0.1 (curve_radius=0) to 0.5 (curve_radius=1.0)
         * Effect: Higher values make curve_radius adjustment more dramatic
         *
         * Example:
         * - curve_radius = 0.0: smoothing = 0.1 (10% of segment)
         * - curve_radius = 0.5: smoothing = 0.3 (30% of segment)
         * - curve_radius = 1.0: smoothing = 0.5 (50% of segment)
         */
        static const float CORNER_SMOOTH_RANGE = 0.4f;

        /**
         * @brief Maximum angle threshold for corner detection (degrees)
         *
         * When curve_radius = 0.0, only corners sharper than 179° are smoothed.
         * This makes smoothing very conservative, affecting only nearly right-angle
         * turns. Nearly straight paths (179-180°) pass through unmodified.
         *
         * Value: 179.0° (almost straight line)
         * Range: 0° to 180° (180° is perfectly straight)
         * Effect: Lower values detect fewer corners (more selective smoothing)
         *
         * Technical Note:
         * The turning angle at a waypoint is 180° minus the angle between
         * the two path segments. A 179° threshold means only 1° turns are smoothed
         * when curve_radius = 0.
         */
        static const float MAX_CORNER_ANGLE_THRESHOLD = 179.0f;

        /**
         * @brief Angle threshold reduction range (degrees)
         *
         * The corner detection threshold decreases linearly as curve_radius increases,
         * making corner detection more aggressive (detects gentler turns as corners).
         *
         * Formula: corner_angle_threshold = MAX_CORNER_ANGLE_THRESHOLD - (curve_radius * CORNER_ANGLE_RANGE)
         *
         * Value: 15.0°
         * Result range: 179° (curve_radius=0) to 164° (curve_radius=1.0)
         * Effect: Higher values make curve_radius more aggressive at detecting corners
         *
         * Example:
         * - curve_radius = 0.0: threshold = 179° (only 1° turns smoothed)
         * - curve_radius = 0.5: threshold = 171.5° (8.5° turns smoothed)
         * - curve_radius = 1.0: threshold = 164° (16° turns smoothed)
         *
         * Performance Impact:
         * Lower threshold values detect more corners, generating more curve points
         * and increasing both computation time and output path length.
         */
        static const float CORNER_ANGLE_RANGE = 15.0f;
    } // namespace smooth_constants

} // namespace pathfinder
#endif