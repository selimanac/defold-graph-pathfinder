#ifndef PATHFINDER_UTILS_H
#define PATHFINDER_UTILS_H

#include "dmarray_include.h"
#include "pathfinder_types.h"
#include <cstdint>

/**
 * @file pathfinder_utils.h
 * @brief Utility functions and helper routines for pathfinding operations
 *
 * This header provides utility functions used throughout the pathfinding engine.
 * These are primarily helper functions for common operations that don't fit into
 * other specific namespaces.
 *
 * Current Utilities:
 * - safe_push(): Auto-growing array insertion for dynamic path storage
 *
 * Design Philosophy:
 * - Inline functions for zero-overhead abstraction
 * - Generic templates where appropriate (C++11 compliant)
 * - No exceptions (uses pre-allocation and growth strategies)
 *
 * Usage Pattern:
 * - Include this header when working with dynamic arrays (dmArray)
 * - Prefer safe_push() over manual capacity checks
 * - Use for path smoothing output buffers
 */

namespace pathfinder
{
    namespace utils
    {
        /**
         * @brief Safely push a Vec2 point to an array, auto-growing if needed
         * @param array Target dmArray<Vec2> to push to
         * @param point Vec2 point to insert
         *
         * Automatically grows the array capacity if it's full before pushing the point.
         * This prevents buffer overflow errors and simplifies code that builds paths
         * or smoothed trajectories.
         *
         * Growth Strategy:
         * - If full: new_capacity = current_capacity + (current_capacity / 2) + 1
         * - Minimum growth: +1 element (handles zero-capacity case)
         * - Growth factor: 1.5x (balance between memory overhead and reallocation frequency)
         *
         * Time Complexity:
         * - O(1) amortized (no growth needed most of the time)
         * - O(n) when growth needed (must copy n elements to new buffer)
         *
         * Memory:
         * - Grows by 50% + 1 element each time capacity is exceeded
         * - Example: 0 → 1 → 2 → 4 → 7 → 11 → 17 → 26 → 40 → 61 → 92...
         *
         * Use Cases:
         * - Building smoothed paths with unknown final size
         * - Accumulating waypoints during path processing
         * - Any scenario where exact capacity is hard to predict
         *
         * Alternative:
         * If exact capacity is known in advance, prefer:
         * @code
         * array.SetCapacity(known_size);
         * for (...) { array.Push(point); }
         * @endcode
         *
         * Thread Safety: Not thread-safe. Do not call concurrently on same array.
         */
        static inline void safe_push(dmArray<Vec2>& array, Vec2 point)
        {
            if (array.Size() >= array.Capacity())
            {
                // Auto-grow by 50% + 1 element
                uint32_t new_capacity = array.Capacity() + (array.Capacity() / 2) + 1;
                array.SetCapacity(new_capacity);
            }
            array.Push(point);
        }

    } // namespace utils
} // namespace pathfinder

#endif
