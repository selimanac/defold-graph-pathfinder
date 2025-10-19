#ifndef PATHFINDER_DISTANCE_CACHE_H
#define PATHFINDER_DISTANCE_CACHE_H

#include "dmarray_include.h"
#include "pathfinder_types.h"
#include <cstdint>

namespace pathfinder
{
    /**
     * @brief Distance caching system for node-to-node Euclidean distances
     *
     * This cache stores precomputed distances between node pairs to avoid
     * redundant distance calculations during pathfinding. It uses a hash table
     * with linear probing for collision resolution and per-node invalidation
     * tracking for efficient cache maintenance.
     *
     * Features:
     * - O(1) average case lookup via hash table with linear probing
     * - Commutative distance storage: distance(A,B) == distance(B,A)
     * - Per-node invalidation chains for O(k) selective invalidation
     * - Power-of-2 sizing for fast modulo operations
     * - Dynamic sizing based on node count (nodeCount * 8)
     * - Batch invalidation support for multiple nodes
     * - Performance statistics tracking (hits/misses)
     *
     * Memory Layout:
     * - Cache entries: dmArray<Entry> with hash table semantics
     * - Node-to-entry mapping: dmArray<uint32_t> for invalidation chains
     * - Each entry: ~20 bytes (from, to, distance, valid, next_entry)
     * - Default size: min(nodeCount * 8, 65536) entries
     *
     * Usage Pattern:
     * 1. Call init() once with expected node count
     * 2. Use cache_get() to fetch/compute/cache distances
     * 3. Call cache_invalidate_node() when nodes move
     * 4. Call resize() if node count changes significantly
     * 5. Use get_stats() to monitor cache performance
     */
    namespace distance_cache
    {
        /**
         * @brief Initialize the distance cache system
         * @param nodeCount Number of nodes in the graph
         *
         * Allocates and initializes the cache based on node count.
         * Cache size is calculated as min(nodeCount * 8, 65536) rounded
         * up to the next power of 2 for efficient hashing.
         *
         * This function:
         * - Allocates cache entries array
         * - Initializes node-to-entry mapping
         * - Resets hit/miss statistics
         * - Sets up hash table mask for fast modulo
         *
         * Time Complexity: O(n) where n = calculated cache size
         * Memory Usage: ~20 bytes per cache entry + 4 bytes per node
         */
        void init(const uint32_t nodeCount);

        /**
         * @brief Resize the cache when node count changes
         * @param newNodeCount New number of nodes in the graph
         *
         * Resizes the cache and attempts to preserve valid entries.
         * If new size differs from current size, valid entries are
         * temporarily copied and reinserted into the new cache.
         *
         * Preservation limits:
         * - Up to 1024 entries: Uses stack allocation for temp storage
         * - More than 1024: Clears cache entirely to avoid heap allocation
         *
         * Time Complexity: O(n) where n = number of valid entries to preserve
         * Note: This operation is relatively expensive, only call when
         * node count changes significantly
         */
        void resize(const uint32_t newNodeCount);

        /**
         * @brief Get distance between two nodes (with caching)
         * @param from First node ID
         * @param to Second node ID
         * @param nodes Pointer to nodes array for position lookup
         * @return Cached or newly computed distance between nodes
         *
         * Performs hash table lookup with linear probing (up to MAX_PROBES
         * attempts). If distance is cached, returns it immediately.
         * If not cached, computes the Euclidean distance, stores it in
         * the cache, and updates the node-to-entry mapping.
         *
         * Distance is stored once but accessible via both (from,to) and
         * (to,from) due to commutative hash function.
         *
         * Special cases:
         * - Returns 0.0f if either node ID is UINT32_MAX (ERROR)
         * - Falls back to direct computation if cache is full
         *
         * Time Complexity: O(1) average case, O(MAX_PROBES) worst case
         * Side Effects: Updates m_Hits or m_Misses counters
         */
        float cache_get(const uint32_t from, const uint32_t to, const dmArray<Node>* nodes);

        /**
         * @brief Invalidate all cached distances involving a specific node
         * @param node_id Node ID whose distances should be invalidated
         *
         * Efficiently invalidates all cache entries containing the specified
         * node by following the node's invalidation chain. This is much faster
         * than scanning the entire cache.
         *
         * This should be called when:
         * - Node position changes (moves)
         * - Node is removed from the graph
         * - Node properties affecting distance change
         *
         * Time Complexity: O(k) where k = number of cached distances for this node
         * Space Complexity: O(1)
         */
        void cache_invalidate_node(const uint32_t node_id);

        /**
         * @brief Batch invalidate cached distances for multiple nodes
         * @param node_ids Array of node IDs to invalidate
         *
         * Efficiently invalidates all cached distances involving any of the
         * specified nodes. Uses a bit array to track already-processed entries
         * to avoid duplicate work when nodes share cached distances.
         *
         * More efficient than calling cache_invalidate_node() in a loop when
         * invalidating many nodes at once.
         *
         * Time Complexity: O(m*k) where m = number of nodes, k = avg entries per node
         * Space Complexity: O(cache_size / 32) bytes on stack for bit array
         */
        void cache_invalidate_nodes(const dmArray<uint32_t>& node_ids);

        /**
         * @brief Clear all cached distances
         *
         * Invalidates all cache entries and resets the node-to-entry mapping.
         * Also resets hit/miss statistics.
         *
         * Use when:
         * - Graph structure changes dramatically
         * - Many nodes have moved
         * - Resetting for benchmarking
         *
         * Time Complexity: O(cache_size + node_count)
         */
        void clear_cache();

        /**
         * @brief Shutdown and deallocate distance cache system
         * 
         * Releases all memory used by the distance cache. After calling this,
         * init() must be called again before using the cache.
         */
        void shutdown();

        /**
         * @brief Get cache usage and performance statistics
         * @param size Output: Total cache size in entries (can be NULL)
         * @param hits Output: Number of cache hits (can be NULL)
         * @param misses Output: Number of cache misses (can be NULL)
         * @param hitRate Output: Hit rate percentage 0-100 (can be NULL)
         *
         * Retrieves performance metrics for cache monitoring and tuning.
         * All parameters are optional (pass NULL to skip).
         *
         * Hit rate calculation: (hits * 100) / (hits + misses)
         * A high hit rate (>80%) indicates good cache sizing.
         * A low hit rate (<50%) may indicate cache is too small or
         * node positions change too frequently.
         *
         * Time Complexity: O(1)
         */
        void get_stats(uint32_t* size, uint32_t* hits, uint32_t* misses, uint32_t* hitRate);
    } // namespace distance_cache
} // namespace pathfinder

#endif