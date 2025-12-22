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
     * - Context-based architecture for user-managed instances
     *
     * Memory Layout:
     * - Cache entries: dmArray<Entry> with hash table semantics
     * - Node-to-entry mapping: dmArray<uint32_t> for invalidation chains
     * - Each entry: ~20 bytes (from, to, distance, valid, next_entry)
     * - Default size: min(nodeCount * 8, 65536) entries
     *
     * Usage Pattern (User-managed context):
     * 1. Create context with create_context(nodeCount)
     * 2. Pass context to all cache functions
     * 3. Call destroy_context(ctx) when done
     */
    namespace distance_cache
    {
        /**
         * @brief Cache entry storing a computed distance between two nodes
         */
        typedef struct Entry
        {
            uint32_t m_From;      // First node ID in the pair
            uint32_t m_To;        // Second node ID in the pair
            float    m_Distance;  // Cached Euclidean distance
            bool     m_Valid;     // False if invalidated
            uint32_t m_NextEntry; // Index to next entry containing the same node (linked list)
        } Entry;

        /**
         * @brief Distance cache context - encapsulates all cache state
         *
         * User-managed context for independent cache instances.
         * Each context maintains its own cache storage, node mappings, and statistics.
         */
        typedef struct DistanceCacheContext
        {
            dmArray<Entry>    m_Cache;         // Cache storage: hash table with linear probing
            dmArray<uint32_t> m_NodeEntryMap;  // Per-node invalidation chains
            uint32_t          m_CacheSize;     // Cache size (power of 2)
            uint32_t          m_CacheMask;     // Bit mask for fast modulo
            uint32_t          m_NodeCount;     // Number of nodes in the graph
            uint32_t          m_Hits;          // Number of cache hits
            uint32_t          m_Misses;        // Number of cache misses
        } DistanceCacheContext;

        /*******************************************/
        // CONTEXT MANAGEMENT
        /*******************************************/

        /**
         * @brief Create a new distance cache context
         * @param nodeCount Number of nodes in the graph
         * @return Pointer to newly created context, or NULL on failure
         *
         * Creates a user-managed context. User is responsible for calling
         * destroy_context() when done.
         */
        DistanceCacheContext* create_context(const uint32_t nodeCount);

        /**
         * @brief Destroy a distance cache context and free all resources
         * @param ctx Context to destroy
         *
         * Releases all memory associated with the context.
         */
        void destroy_context(DistanceCacheContext* ctx);

        /*******************************************/
        // CACHE OPERATIONS
        /*******************************************/
        /*******************************************/
        // CACHE OPERATIONS
        /*******************************************/

        /**
         * @brief Resize the cache when node count changes
         * @param ctx Context to resize
         * @param newNodeCount New number of nodes in the graph
         */
        void resize(DistanceCacheContext* ctx, const uint32_t newNodeCount);

        /**
         * @brief Get distance between two nodes (with caching)
         * @param ctx Context to use
         * @param from First node ID
         * @param to Second node ID
         * @param nodes Pointer to nodes array for position lookup
         * @return Cached or newly computed distance between nodes
         */
        float cache_get(DistanceCacheContext* ctx, const uint32_t from, const uint32_t to, const dmArray<Node>* nodes);

        /**
         * @brief Get distance between two Vec2 positions (with caching)
         * @param ctx Context to use
         * @param from_key First position key (e.g., cell index)
         * @param to_key Second position key (e.g., goal marker)
         * @param from_pos First position
         * @param to_pos Second position
         * @return Cached or newly computed distance between positions
         *
         * This variant supports caching distances between arbitrary Vec2 positions
         * using integer keys. Useful for navmesh A* where portal midpoints are
         * computed dynamically but are stable for a given cell pair.
         *
         * Example usage (navmesh A*):
         *   float h = cache_get_positions(ctx, current_cell, UINT32_MAX, portal_midpoint, goal_pos);
         */
        float cache_get_positions(DistanceCacheContext* ctx, const uint32_t from_key, const uint32_t to_key, const Vec2 from_pos, const Vec2 to_pos);

        /**
         * @brief Invalidate all cached distances involving a specific node
         * @param ctx Context to use
         * @param node_id Node ID whose distances should be invalidated
         */
        void cache_invalidate_node(DistanceCacheContext* ctx, const uint32_t node_id);

        /**
         * @brief Batch invalidate cached distances for multiple nodes
         * @param ctx Context to use
         * @param node_ids Array of node IDs to invalidate
         */
        void cache_invalidate_nodes(DistanceCacheContext* ctx, const dmArray<uint32_t>& node_ids);

        /**
         * @brief Clear all cached distances
         * @param ctx Context to clear
         */
        void clear_cache(DistanceCacheContext* ctx);

        /**
         * @brief Get cache usage and performance statistics
         * @param ctx Context to query
         * @param size Output: Total cache size in entries (can be NULL)
         * @param hits Output: Number of cache hits (can be NULL)
         * @param misses Output: Number of cache misses (can be NULL)
         * @param hitRate Output: Hit rate percentage 0-100 (can be NULL)
         */
        void get_stats(DistanceCacheContext* ctx, uint32_t* size, uint32_t* hits, uint32_t* misses, uint32_t* hitRate);
    } // namespace distance_cache
} // namespace pathfinder

#endif