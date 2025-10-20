#ifndef PATHFINDER_CACHE_H
#define PATHFINDER_CACHE_H

#include "dmarray_include.h"
#include "dmhashtable_include.h"
#include <pathfinder_heap.h>
#include "pathfinder_types.h"
#include <pathfinder_math.h>
#include <cstdint>

namespace pathfinder
{
    /**
     * @brief Path caching system with LRU eviction and version-based invalidation
     *
     * This cache stores previously computed paths to avoid redundant pathfinding
     * calculations. It uses a dual hash table approach (regular + projected paths)
     * with LRU eviction when full.
     *
     * Features:
     * - O(1) average case lookup via hash tables
     * - Fine-grained invalidation using version tracking
     * - Support for both node-to-node and point-to-node (projected) paths
     * - LRU eviction policy when cache is full
     * - Memory efficient with pre-allocated storage and free list
     *
     * Version-based invalidation:
     * - Node version: Incremented when nodes move or change state
     * - Edge version: Incremented when edges are added/removed/modified
     * - Per-node tracking: Identifies which specific nodes affect cached paths
     */
    namespace cache
    {
        /**
         * @brief Initialize the path cache system
         * @param cache_size Maximum number of paths to cache
         * @param max_cache_path_length Maximum length of any cached path
         */
        void init(const uint32_t cache_size, const uint32_t max_cache_path_length);

        /**
         * @brief Search for a cached path between two nodes
         * @param start_id Starting node ID
         * @param goal_id Goal node ID
         * @param out_path Output array to receive the cached path
         * @param max_path Maximum number of nodes to copy
         * @return Number of nodes in path, or INVALID_ID if not found/invalid
         */
        uint32_t find_path(const uint32_t start_id, const uint32_t goal_id, dmArray<uint32_t>* out_path, const uint32_t max_path);

        /**
         * @brief Add or update a path in the cache
         * @param start_id Starting node ID
         * @param goal_id Goal node ID
         * @param path Array containing the path nodes
         * @param length Number of nodes in the path
         */
        void add_path(const uint32_t start_id, const uint32_t goal_id, const dmArray<uint32_t>* path, const uint32_t length);

        /**
         * @brief Search for a cached projected path from a point to a node
         * @param start_point Starting 2D position (not necessarily a node)
         * @param goal_id Goal node ID
         * @param out_path Output array to receive the cached path
         * @param max_path Maximum number of nodes to copy
         * @param out_entry_point Output: point where agent enters the graph
         * @return Number of nodes in path, or INVALID_ID if not found/invalid
         */
        uint32_t find_projected_path(const Vec2 start_point, const uint32_t goal_id, dmArray<uint32_t>* out_path, const uint32_t max_path, Vec2* out_entry_point);

        /**
         * @brief Add or update a projected path in the cache
         * @param start_point Starting 2D position (not necessarily a node)
         * @param goal_id Goal node ID
         * @param path Array containing the path nodes
         * @param length Number of nodes in the path
         * @param entry_point Point where the agent enters the graph
         */
        void add_projected_path(const Vec2 start_point, const uint32_t goal_id, const dmArray<uint32_t>* path, const uint32_t length, const Vec2 entry_point);

        /**
         * @brief Invalidate all cached paths containing a specific node
         * @param node_id Node ID that has been modified
         */
        void invalidate_node(const uint32_t node_id);

        /**
         * @brief Invalidate all cached paths using a specific edge
         * @param from First node of the edge
         * @param to Second node of the edge
         */
        void invalidate_edge(const uint32_t from, const uint32_t to);

        /**
         * @brief Clear all cached paths
         */
        void clear_cache();

        /**
         * @brief Shutdown and deallocate cache system
         * 
         * Releases all memory used by the cache system. After calling this,
         * init() must be called again before using the cache.
         */
        void shutdown();

        /**
         * @brief Get cache usage and performance statistics
         * @param entries Output: Number of currently cached paths (can be NULL)
         * @param capacity Output: Maximum cache capacity (can be NULL)
         * @param hit_rate Output: Cache hit rate percentage 0-100 (can be NULL)
         */
        void get_cache_stats(uint32_t* entries, uint32_t* capacity, uint32_t* hit_rate);
    } // namespace cache
} // namespace pathfinder

#endif