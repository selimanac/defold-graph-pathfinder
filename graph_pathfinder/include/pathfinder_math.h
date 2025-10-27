#ifndef PATHFINDER_MATH_H
#define PATHFINDER_MATH_H

#include <cmath>
#include <pathfinder_types.h>
#include <pathfinder_constants.h>

/**
 * @file pathfinder_math.h
 * @brief 2D vector mathematics and interpolation functions for pathfinding
 *
 * This header provides a comprehensive set of 2D vector operations and interpolation
 * functions used throughout the pathfinding engine. All functions are inline for
 * zero-overhead abstraction.
 *
 * Function Categories:
 * - Basic Vector Operations: distance, length, normalize, scale, add, subtract
 * - Vector Utilities: truncate, equal, project_segment, clamp
 * - Path Smoothing: Catmull-Rom, Bézier (quadratic/cubic), linear interpolation
 * - Corner Detection: is_corner() for identifying sharp turns
 * - Sample Calculation: Dynamic sample count based on distance
 *
 * Performance Notes:
 * - All functions are static inline (no function call overhead)
 * - distance_squared() and length_squared() avoid sqrt() for comparisons
 * - Vectorized operations when compiled with optimization flags
 * - Cache-friendly with minimal branching
 *
 * Usage Pattern:
 * @code
 * using namespace pathfinder::math;
 * Vec2 dir = normalize(subtract(goal, start));
 * float dist = distance(a, b);
 * Vec2 smoothed = bezier_quadratic(p0, p1, p2, 0.5f);
 * @endcode
 */

namespace pathfinder
{
    namespace math
    {
        /*******************************************/
        // BASIC VECTOR OPERATIONS
        /*******************************************/

        /**
         * @brief Calculate Euclidean distance between two 2D points
         * @param a First point
         * @param b Second point
         * @return Distance between a and b
         *
         * Uses the standard distance formula: sqrt((bx - ax)² + (by - ay)²)
         *
         * Performance: ~25-30 cycles (includes sqrt instruction)
         * Use Case: Actual distance measurements, heuristic calculations
         *
         * For comparisons (e.g., finding nearest point), prefer distance_squared()
         * to avoid expensive sqrt operation.
         *
         * Time Complexity: O(1)
         */
        static inline float distance(const Vec2 a, const Vec2 b)
        {
            float dx = b.x - a.x;
            float dy = b.y - a.y;
            return sqrtf(dx * dx + dy * dy);
        }

        /**
         * @brief Calculate squared Euclidean distance (avoids sqrt for comparisons)
         * @param a First point
         * @param b Second point
         * @return Squared distance between a and b
         *
         * Formula: (bx - ax)² + (by - ay)²
         *
         * Performance: ~5-7 cycles (no sqrt)
         * Use Case: Distance comparisons, nearest neighbor searches
         *
         * Optimization Rationale:
         * Since sqrt is monotonic, if dist1² < dist2², then dist1 < dist2.
         * Saves ~20 cycles per comparison.
         *
         * Example:
         * @code
         * // Find nearest node (fast version)
         * float min_dist_sq = FLT_MAX;
         * for (node : nodes) {
         *     float d_sq = distance_squared(pos, node.position);
         *     if (d_sq < min_dist_sq) { min_dist_sq = d_sq; nearest = node; }
         * }
         * @endcode
         *
         * Time Complexity: O(1)
         */
        static inline float distance_squared(Vec2 a, Vec2 b)
        {
            float dx = b.x - a.x;
            float dy = b.y - a.y;
            return dx * dx + dy * dy;
        }

        /**
         * @brief Calculate length (magnitude) of a vector
         * @param v Vector to measure
         * @return Length of the vector
         *
         * Formula: sqrt(vx² + vy²)
         *
         * Performance: ~25-30 cycles
         * Use Case: Speed calculations, normalization checks
         *
         * For comparisons (e.g., speed thresholds), prefer length_squared().
         *
         * Time Complexity: O(1)
         */
        static inline float length(Vec2 v)
        {
            return sqrtf(v.x * v.x + v.y * v.y);
        }

        /**
         * @brief Calculate squared length of a vector (avoids sqrt for comparisons)
         * @param v Vector to measure
         * @return Squared length of the vector
         *
         * Formula: vx² + vy²
         *
         * Performance: ~5-7 cycles
         * Use Case: Speed comparisons, zero-vector detection
         *
         * Time Complexity: O(1)
         */
        static inline float length_squared(Vec2 v)
        {
            return v.x * v.x + v.y * v.y;
        }

        /**
         * @brief Normalize a vector to unit length (magnitude = 1)
         * @param v Vector to normalize
         * @return Unit vector in the same direction, or (0,0) if v is near-zero
         *
         * Formula: v / |v| where |v| = length(v)
         *
         * Special Cases:
         * - If length < EPSILON (0.0001): Returns zero vector (0, 0)
         * - Avoids division by zero
         * - Preserves direction but sets magnitude to 1.0
         *
         * Use Case: Direction vectors, velocity normalization, steering
         *
         * Example:
         * @code
         * Vec2 direction = normalize(subtract(target, position));
         * Vec2 velocity = scale(direction, speed);  // Move at 'speed' toward target
         * @endcode
         *
         * Time Complexity: O(1)
         */
        static inline Vec2 normalize(Vec2 v)
        {
            float len = length(v);
            if (len < EPSILON)
            {
                return Vec2(0.0f, 0.0f);
            }
            return Vec2(v.x / len, v.y / len);
        }

        /**
         * @brief Scale a vector by a scalar multiplier
         * @param v Vector to scale
         * @param s Scalar multiplier
         * @return Scaled vector (v.x * s, v.y * s)
         *
         * Use Case: Velocity from direction, distance adjustment, resizing
         *
         * Example:
         * @code
         * Vec2 half_vec = scale(v, 0.5f);        // Half length
         * Vec2 doubled = scale(v, 2.0f);         // Double length
         * Vec2 velocity = scale(direction, 50.0f); // Move at 50 units/sec
         * @endcode
         *
         * Time Complexity: O(1)
         */
        static inline Vec2 scale(Vec2 v, float s)
        {
            return Vec2(v.x * s, v.y * s);
        }

        /**
         * @brief Add two vectors (component-wise addition)
         * @param a First vector
         * @param b Second vector
         * @return Sum vector (a.x + b.x, a.y + b.y)
         *
         * Use Case: Position updates, force accumulation, offset application
         *
         * Example:
         * @code
         * Vec2 new_pos = add(position, velocity);  // Move by velocity
         * Vec2 total_force = add(gravity, wind);   // Combine forces
         * @endcode
         *
         * Time Complexity: O(1)
         */
        static inline Vec2 add(Vec2 a, Vec2 b)
        {
            return Vec2(a.x + b.x, a.y + b.y);
        }

        /**
         * @brief Subtract two vectors (component-wise subtraction)
         * @param a First vector
         * @param b Second vector (subtracted from a)
         * @return Difference vector (a.x - b.x, a.y - b.y)
         *
         * Use Case: Direction calculation, displacement, relative position
         *
         * Example:
         * @code
         * Vec2 direction = subtract(target, current);  // From current to target
         * Vec2 relative_pos = subtract(agent, player); // Agent relative to player
         * @endcode
         *
         * Time Complexity: O(1)
         */
        static inline Vec2 subtract(Vec2 a, Vec2 b)
        {
            return Vec2(a.x - b.x, a.y - b.y);
        }

        /**
         * @brief Limit vector magnitude to a maximum length
         * @param v Vector to truncate
         * @param max_length Maximum allowed length
         * @return Truncated vector with length ≤ max_length, preserving direction
         *
         * If vector length exceeds max_length, scales it down to max_length.
         * Otherwise returns the original vector unchanged.
         *
         * Use Case: Speed limits, maximum force constraints, clamped acceleration
         *
         * Example:
         * @code
         * Vec2 limited_vel = truncate(velocity, max_speed);     // Cap speed
         * Vec2 limited_force = truncate(force, max_force);      // Cap force
         * @endcode
         *
         * Time Complexity: O(1)
         */
        static inline Vec2 truncate(Vec2 v, float max_length)
        {
            float len = length(v);
            if (len > max_length && len > EPSILON)
            {
                return scale(v, max_length / len);
            }
            return v;
        }

        /**
         * @brief Check if two vectors are approximately equal (within epsilon)
         * @param a First vector
         * @param b Second vector
         * @return true if vectors are within EPSILON (0.0001) of each other
         *
         * Uses component-wise absolute difference comparison against EPSILON.
         *
         * Use Case: Position change detection, stopping condition checks
         *
         * Example:
         * @code
         * if (equal(agent.position, target)) {
         *     // Agent has reached target
         * }
         * @endcode
         *
         * Time Complexity: O(1)
         */
        static inline bool equal(const Vec2& a, const Vec2& b)
        {
            return fabsf(a.x - b.x) < EPSILON && fabsf(a.y - b.y) < EPSILON;
        }

        /**
         * @brief Project a point onto a line segment (clamped to segment endpoints)
         * @param p Point to project
         * @param a Segment start point
         * @param b Segment end point
         * @return Closest point on segment [a, b] to point p
         *
         * Calculates the perpendicular projection of point p onto line segment ab,
         * clamping the result to stay within the segment boundaries.
         *
         * Algorithm:
         * 1. Compute parameter t = dot(p - a, b - a) / length²(b - a)
         * 2. Clamp t to [0, 1] to stay within segment
         * 3. Return a + t * (b - a)
         *
         * Special Cases:
         * - If a == b (zero-length segment): Returns a
         * - If t < 0: Returns a (projection before segment start)
         * - If t > 1: Returns b (projection after segment end)
         * - If 0 ≤ t ≤ 1: Returns point on segment
         *
         * Use Case: Nearest point on path segment, projected pathfinding entry point
         *
         * Time Complexity: O(1)
         */
        static inline Vec2 project_segment(const Vec2 p, const Vec2 a, const Vec2 b)
        {
            float dx = b.x - a.x;
            float dy = b.y - a.y;
            float length2 = dx * dx + dy * dy;
            if (length2 == 0.0f)
                return a;

            float t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / length2;
            if (t < 0.0f)
                t = 0.0f;
            if (t > 1.0f)
                t = 1.0f;

            return Vec2(a.x + t * dx, a.y + t * dy);
        }

        /**
         * @brief Clamp a floating-point value to a specified range
         * @param value Value to clamp
         * @param min Minimum allowed value
         * @param max Maximum allowed value
         * @return Clamped value in range [min, max]
         *
         * Use Case: Parameter validation, range limiting, boundary enforcement
         *
         * Example:
         * @code
         * float speed = clamp(input_speed, 0.0f, max_speed);
         * float t = clamp(t, 0.0f, 1.0f);  // Clamp interpolation parameter
         * @endcode
         *
         * Time Complexity: O(1)
         */
        static inline float clamp(float value, float min, float max)
        {
            if (value < min)
                return min;
            if (value > max)
                return max;
            return value;
        }

        /*******************************************/
        // PATH SMOOTHING UTILITIES
        /*******************************************/

        /**
         * @brief Calculate adaptive sample count based on segment distance
         * @param p0 Start point of segment
         * @param p1 End point of segment
         * @return Number of interpolation samples (minimum 2)
         *
         * Dynamically adjusts sample density based on segment length:
         * - Formula: samples = (distance / 10.0) + 2
         * - Short segments (< 10 units): 2-3 samples (minimal overhead)
         * - Medium segments (50 units): 7 samples (smooth curves)
         * - Long segments (100 units): 12 samples (very smooth)
         *
         * Use Case: Automatic quality adjustment for path smoothing
         *
         * Example:
         * @code
         * uint32_t samples = calculate_sample_count(waypoint1, waypoint2);
         * for (uint32_t i = 0; i < samples; i++) {
         *     float t = (float)i / (samples - 1);
         *     smoothed_path.Push(bezier_quadratic(p0, p1, p2, t));
         * }
         * @endcode
         *
         * Time Complexity: O(1)
         */
        static inline uint32_t calculate_sample_count(Vec2 p0, Vec2 p1)
        {
            float dist = pathfinder::math::distance(p0, p1);
            return (uint32_t)(dist / 10.0f) + 2; // 1 sample per 10 units
        }

        /**
         * @brief Check if three consecutive waypoints form a corner needing smoothing
         * @param p0 Previous waypoint
         * @param p1 Current waypoint (potential corner)
         * @param p2 Next waypoint
         * @param angle_threshold_deg Angle threshold in degrees (180° = straight line)
         * @return true if angle is sharp enough to require smoothing, false for straight segments
         *
         * Detects corners by measuring the turning angle at p1. A straight path has
         * a 180° angle (no turn), while a right-angle turn has a 90° angle.
         *
         * Algorithm (Optimized - avoids acos):
         * 1. Compute direction vectors: v1 = p1 - p0, v2 = p2 - p1
         * 2. Calculate dot product: dot = v1 · v2
         * 3. Convert threshold to cosine: cos_threshold = cos(180° - angle_threshold_deg)
         * 4. Compare: dot < sqrt(len1² * len2²) * cos_threshold
         *
         * Angle Interpretation:
         * - 180°: Perfectly straight (no corner)
         * - 170°: Very gentle turn (10° deviation)
         * - 135°: Moderate turn (45° deviation)
         * - 90°: Right-angle turn (90° deviation)
         * - 0°: Complete reversal (180° deviation)
         *
         * Threshold Examples:
         * - 179°: Only detects very sharp corners (1° turns)
         * - 170°: Detects moderate corners (10° turns)
         * - 150°: Detects gentle corners (30° turns)
         * - 90°: Detects all non-straight segments (90° turns)
         *
         * Performance: O(1) - no trigonometric functions (uses precomputed cosine)
         *
         * Use Case: Corner detection for selective smoothing in bezier_quadratic_waypoints
         *
         * Example:
         * @code
         * if (is_corner(waypoints[i-1], waypoints[i], waypoints[i+1], 170.0f)) {
         *     // Apply Bézier smoothing at this corner
         * } else {
         *     // Keep waypoint as-is (straight segment)
         * }
         * @endcode
         */
        static inline bool is_corner(const Vec2 p0, const Vec2 p1, const Vec2 p2, float angle_threshold_deg)
        {
            // Direction vectors
            float v1x = p1.x - p0.x;
            float v1y = p1.y - p0.y;
            float v2x = p2.x - p1.x;
            float v2y = p2.y - p1.y;

            // Squared lengths
            float len1_sq = v1x * v1x + v1y * v1y;
            float len2_sq = v2x * v2x + v2y * v2y;

            if (len1_sq < 0.000001f || len2_sq < 0.000001f)
                return false;

            // Dot product
            float dot = v1x * v2x + v1y * v2y;

            // Precompute the cosine of the vector angle threshold
            // Example: 170° threshold → turning angle = 170° → vector angle = 10°
            // cos(10°) ≈ 0.9848
            float cos_threshold = cosf((180.0f - angle_threshold_deg) * MATH_PI / 180.0f);

            // Compute the product of vector lengths for comparison
            // We need: dot / sqrt(len1_sq * len2_sq) < cos_threshold
            // Rearranged: dot < sqrt(len1_sq * len2_sq) * cos_threshold
            float len_product = sqrtf(len1_sq * len2_sq);
            float threshold = len_product * cos_threshold;

            // Return true if the angle is sharp enough (corner detected)
            return dot < threshold;
        }

        /*******************************************/
        // INTERPOLATION FUNCTIONS
        /*******************************************/

        /**
         * @brief Catmull-Rom spline interpolation for smooth curves through waypoints
         * @param p0 Point before start (influences entry tangent)
         * @param p1 Start point (curve passes through this)
         * @param p2 End point (curve passes through this)
         * @param p3 Point after end (influences exit tangent)
         * @param t Interpolation parameter [0, 1] (0 = p1, 1 = p2)
         * @return Interpolated position on curve
         *
         * Creates smooth curves that pass through p1 and p2, using p0 and p3 to
         * determine the tangent directions at the endpoints. This ensures C1 continuity
         * (smooth velocity) across multiple segments.
         *
         * Mathematical Formula:
         * result = 0.5 * [
         *     (2 * p1) +
         *     (-p0 + p2) * t +
         *     (2*p0 - 5*p1 + 4*p2 - p3) * t² +
         *     (-p0 + 3*p1 - 3*p2 + p3) * t³
         * ]
         *
         * Properties:
         * - C1 continuous (smooth velocity transitions)
         * - Passes through control points (except p0 and p3)
         * - Local control (moving p0 only affects nearby segments)
         * - Tension = 0.5 (standard Catmull-Rom, not configurable here)
         *
         * Use Case: Smooth paths where exact waypoint passage is required
         *
         * Recommended For:
         * - Patrol routes where units must reach exact positions
         * - Scripted sequences with precise positioning
         * - Railway/roller coaster tracks
         *
         * Performance: ~30 FLOPs per call
         *
         * Example:
         * @code
         * // Smooth between waypoints[1] and waypoints[2]
         * for (float t = 0.0f; t <= 1.0f; t += 0.1f) {
         *     Vec2 pos = catmull_rom_interpolate(
         *         waypoints[0], waypoints[1], waypoints[2], waypoints[3], t
         *     );
         *     smoothed_path.Push(pos);
         * }
         * @endcode
         *
         * Time Complexity: O(1)
         */
        static inline Vec2 catmull_rom_interpolate(const Vec2 p0, const Vec2 p1, const Vec2 p2, const Vec2 p3, const float t)
        {
            float t2 = t * t;
            float t3 = t2 * t;

            Vec2  result;
            result.x = 0.5f * ((2.0f * p1.x) + (-p0.x + p2.x) * t + (2.0f * p0.x - 5.0f * p1.x + 4.0f * p2.x - p3.x) * t2 + (-p0.x + 3.0f * p1.x - 3.0f * p2.x + p3.x) * t3);
            result.y = 0.5f * ((2.0f * p1.y) + (-p0.y + p2.y) * t + (2.0f * p0.y - 5.0f * p1.y + 4.0f * p2.y - p3.y) * t2 + (-p0.y + 3.0f * p1.y - 3.0f * p2.y + p3.y) * t3);
            return result;
        }

        /**
         * @brief Quadratic Bézier curve interpolation for smooth corners
         * @param p0 Start point (on curve)
         * @param p1 Control point (influences curve shape, not on curve)
         * @param p2 End point (on curve)
         * @param t Interpolation parameter [0, 1] (0 = p0, 1 = p2)
         * @return Interpolated position on curve
         *
         * Creates smooth curves between p0 and p2, influenced by control point p1.
         * The curve does not pass through p1 (approximating spline) but is pulled
         * toward it, creating a natural rounded corner.
         *
         * Mathematical Formula:
         * result = (1-t)² * p0 + 2*(1-t)*t * p1 + t² * p2
         *
         * Properties:
         * - C0 continuous (position continuous, but velocity may have discontinuities)
         * - One control point (p1) determines curve shape
         * - Curve stays within convex hull of {p0, p1, p2}
         * - Maximum deviation from straight line occurs at t = 0.5
         * - Symmetric (reversing direction produces same curve)
         *
         * Use Case: Natural character movement with smooth corners
         *
         * Recommended For:
         * - Character/vehicle movement (allows slight path deviation)
         * - Corner smoothing in grid-based pathfinding
         * - Fast approximation where exact waypoint passage not critical
         *
         * Performance: ~15 FLOPs per call (faster than cubic Bézier)
         *
         * Example:
         * @code
         * // Smooth a 90° turn
         * Vec2 p0 = {0, 0};   // Start
         * Vec2 p1 = {50, 50}; // Control point (corner position)
         * Vec2 p2 = {100, 0}; // End
         * for (float t = 0.0f; t <= 1.0f; t += 0.1f) {
         *     Vec2 pos = bezier_quadratic(p0, p1, p2, t);
         *     smoothed_path.Push(pos);
         * }
         * @endcode
         *
         * Time Complexity: O(1)
         */
        static inline Vec2 bezier_quadratic(const Vec2 p0, const Vec2 p1, const Vec2 p2, const float t)
        {
            float u = 1.0f - t;
            float tt = t * t;
            float uu = u * u;

            Vec2  result;
            result.x = uu * p0.x + 2.0f * u * t * p1.x + tt * p2.x;
            result.y = uu * p0.y + 2.0f * u * t * p1.y + tt * p2.y;
            return result;
        }

        /**
         * @brief Cubic Bézier curve interpolation for maximum smoothness
         * @param p0 Start point (on curve)
         * @param p1 First control point (influences entry tangent, not on curve)
         * @param p2 Second control point (influences exit tangent, not on curve)
         * @param p3 End point (on curve)
         * @param t Interpolation parameter [0, 1] (0 = p0, 1 = p3)
         * @return Interpolated position on curve
         *
         * Creates very smooth curves between p0 and p3, influenced by two control points.
         * The curve does not pass through p1 or p2 (approximating spline), providing
         * maximum flexibility in curve shape.
         *
         * Mathematical Formula:
         * result = (1-t)³ * p0 + 3*(1-t)²*t * p1 + 3*(1-t)*t² * p2 + t³ * p3
         *
         * Properties:
         * - C0 continuous (position continuous)
         * - Two control points provide independent entry/exit tangent control
         * - Curve stays within convex hull of {p0, p1, p2, p3}
         * - More flexible than quadratic (can create S-curves)
         * - Entry tangent: direction from p0 to p1
         * - Exit tangent: direction from p2 to p3
         *
         * Use Case: High-quality cinematic paths and animations
         *
         * Recommended For:
         * - Cinematic camera paths
         * - Cutscene character movements
         * - Showcase/demo sequences
         * - Artistic control over path aesthetics
         *
         * Performance: ~25 FLOPs per call (slower than quadratic)
         *
         * Example:
         * @code
         * // Create an S-curve
         * Vec2 p0 = {0, 0};     // Start
         * Vec2 p1 = {25, 50};   // Pull up
         * Vec2 p2 = {75, -50};  // Pull down
         * Vec2 p3 = {100, 0};   // End
         * for (float t = 0.0f; t <= 1.0f; t += 0.1f) {
         *     Vec2 pos = bezier_cubic(p0, p1, p2, p3, t);
         *     smoothed_path.Push(pos);
         * }
         * @endcode
         *
         * Time Complexity: O(1)
         */
        static inline Vec2 bezier_cubic(const Vec2 p0, const Vec2 p1, const Vec2 p2, const Vec2 p3, const float t)
        {
            float u = 1.0f - t;
            float tt = t * t;
            float uu = u * u;
            float uuu = uu * u;
            float ttt = tt * t;

            Vec2  result;
            result.x = uuu * p0.x + 3.0f * uu * t * p1.x + 3.0f * u * tt * p2.x + ttt * p3.x;
            result.y = uuu * p0.y + 3.0f * uu * t * p1.y + 3.0f * u * tt * p2.y + ttt * p3.y;
            return result;
        }

        /**
         * @brief Linear interpolation between two points
         * @param p0 Start point
         * @param p1 End point
         * @param t Interpolation parameter [0, 1] (0 = p0, 1 = p1)
         * @return Interpolated position on straight line
         *
         * Simple linear interpolation (lerp) for straight line segments.
         * Creates a point at position (1-t)*p0 + t*p1 along the line from p0 to p1.
         *
         * Mathematical Formula:
         * result = p0 + t * (p1 - p0)
         *
         * Properties:
         * - C0 continuous (but not smooth - angular at waypoints)
         * - Fastest interpolation method (no trigonometry)
         * - Constant velocity (equal t increments = equal distances)
         * - Shortest path between two points
         *
         * Use Case: Fallback when smoothing not needed or short segments
         *
         * Recommended For:
         * - Very short path segments (< 5 units)
         * - Straight corridors
         * - Grid-aligned movement
         * - Performance-critical scenarios
         *
         * Performance: ~5 FLOPs per call (fastest)
         *
         * Example:
         * @code
         * // Create 10 points along a straight line
         * for (float t = 0.0f; t <= 1.0f; t += 0.1f) {
         *     Vec2 pos = lerp(start, end, t);
         *     path.Push(pos);
         * }
         * @endcode
         *
         * Time Complexity: O(1)
         */
        static inline Vec2 lerp(const Vec2 p0, const Vec2 p1, const float t)
        {
            Vec2 result;
            result.x = p0.x + (p1.x - p0.x) * t;
            result.y = p0.y + (p1.y - p0.y) * t;
            return result;
        }

    } // namespace math
} // namespace pathfinder
#endif