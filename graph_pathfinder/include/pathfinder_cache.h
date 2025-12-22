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
     * - Context-based architecture for user-managed instances
     *
     * Version-based invalidation:
     * - Node version: Incremented when nodes move or change state
     * - Edge version: Incremented when edges are added/removed/modified
     * - Per-node tracking: Identifies which specific nodes affect cached paths
     */
    namespace cache
    {
        // Forward declarations for internal types
        typedef struct PathKey PathKey;
        typedef struct PathResult PathResult;

        /**
         * @brief Cache context - encapsulates all cache state
         *
         * User-managed context for independent cache instances.
         * Each context maintains its own cache storage, hash tables, and heap context.
         */
        typedef struct CacheContext
        {
            dmHashTable64<PathResult*> m_PathCache;          // Hash table for regular paths
            dmHashTable64<PathResult*> m_ProjPathCache;      // Hash table for projected paths
            dmArray<PathResult>        m_Entries;            // Path storage
            dmArray<uint32_t>          m_FreeList;           // List of free indices
            uint32_t                   m_CacheSize;          // Maximum number of paths to cache
            uint32_t                   m_MaxCachePathLength; // Maximum length of any cached path
            uint32_t                   m_Timestamp;          // LRU timestamp counter
            uint32_t                   m_Hits;               // Cache hit counter
            uint32_t                   m_Misses;             // Cache miss counter
            heap::HeapContext*         m_HeapContext;        // Heap context for version tracking
        } CacheContext;

        /*******************************************/
        // CONTEXT MANAGEMENT
        /*******************************************/

        /**
         * @brief Create a new cache context
         * @param cache_size Maximum number of paths to cache
         * @param max_cache_path_length Maximum length of any cached path
         * @param heap_context Heap context to use for version tracking
         * @return Pointer to newly created context, or NULL on failure
         *
         * Creates a user-managed context. User is responsible for calling
         * destroy_context() when done.
         */
        CacheContext* create_context(const uint32_t cache_size, const uint32_t max_cache_path_length, heap::HeapContext* heap_context);

        /**
         * @brief Destroy a cache context and free all resources
         * @param ctx Context to destroy
         *
         * Releases all memory associated with the context.
         */
        void destroy_context(CacheContext* ctx);

        /*******************************************/
        // CACHE OPERATIONS
        /*******************************************/

        /**
         * @brief Search for a cached path between two nodes
         * @param ctx Context to use
         * @param start_id Starting node ID
         * @param goal_id Goal node ID
         * @param out_path Output array to receive the cached path
         * @param max_path Maximum number of nodes to copy
         * @return Number of nodes in path, or INVALID_ID if not found/invalid
         */
        uint32_t find_path(CacheContext* ctx, const uint32_t start_id, const uint32_t goal_id, dmArray<uint32_t>* out_path, const uint32_t max_path);

        /**
         * @brief Add or update a path in the cache
         * @param ctx Context to use
         * @param start_id Starting node ID
         * @param goal_id Goal node ID
         * @param path Array containing the path nodes
         * @param length Number of nodes in the path
         */
        void add_path(CacheContext* ctx, const uint32_t start_id, const uint32_t goal_id, const dmArray<uint32_t>* path, const uint32_t length);

        /**
         * @brief Search for a cached projected path from a point to a node
         * @param ctx Context to use
         * @param start_point Starting 2D position (not necessarily a node)
         * @param goal_id Goal node ID
         * @param out_path Output array to receive the cached path
         * @param max_path Maximum number of nodes to copy
         * @param out_entry_point Output: point where agent enters the graph
         * @return Number of nodes in path, or INVALID_ID if not found/invalid
         */
        uint32_t find_projected_path(CacheContext* ctx, const Vec2 start_point, const uint32_t goal_id, dmArray<uint32_t>* out_path, const uint32_t max_path, Vec2* out_entry_point);

        /**
         * @brief Add or update a projected path in the cache
         * @param ctx Context to use
         * @param start_point Starting 2D position (not necessarily a node)
         * @param goal_id Goal node ID
         * @param path Array containing the path nodes
         * @param length Number of nodes in the path
         * @param entry_point Point where the agent enters the graph
         */
        void add_projected_path(CacheContext* ctx, const Vec2 start_point, const uint32_t goal_id, const dmArray<uint32_t>* path, const uint32_t length, const Vec2 entry_point);

        /**
         * @brief Search for a cached path with exit point projection
         * @param ctx Context to use
         * @param start_point Starting 2D position (used for hashing)
         * @param end_point Ending 2D position (not necessarily a node)
         * @param out_path Output array to receive the cached path
         * @param max_path Maximum number of nodes to copy
         * @param out_entry_point Output: point where agent enters the graph (optional, can be NULL)
         * @param out_exit_point Output: point where agent exits the graph to reach end_point
         * @return Number of nodes in path, or INVALID_ID if not found/invalid
         */
        uint32_t find_path_with_exit(CacheContext* ctx, const Vec2 start_point, const Vec2 end_point, dmArray<uint32_t>* out_path, const uint32_t max_path, Vec2* out_entry_point, Vec2* out_exit_point);

        /**
         * @brief Add or update a path with exit point in the cache
         * @param ctx Context to use
         * @param start_point Starting 2D position (can be node position or arbitrary position)
         * @param end_point Ending 2D position (arbitrary position)
         * @param path Array containing the path nodes
         * @param length Number of nodes in the path
         * @param entry_point Point where the agent enters the graph (if starting from arbitrary position)
         * @param exit_point Point where the agent exits the graph to reach end_point
         */
        void add_path_with_exit(CacheContext* ctx, const Vec2 start_point, const Vec2 end_point, const dmArray<uint32_t>* path, const uint32_t length, const Vec2 entry_point, const Vec2 exit_point);

        /**
         * @brief Invalidate all cached paths containing a specific node
         * @param ctx Context to use
         * @param node_id Node ID that has been modified
         */
        void invalidate_node(CacheContext* ctx, const uint32_t node_id);

        /**
         * @brief Invalidate all cached paths using a specific edge
         * @param ctx Context to use
         * @param from First node of the edge
         * @param to Second node of the edge
         */
        void invalidate_edge(CacheContext* ctx, const uint32_t from, const uint32_t to);

        /**
         * @brief Clear all cached paths
         * @param ctx Context to clear
         */
        void clear_cache(CacheContext* ctx);

        /**
         * @brief Get cache usage and performance statistics
         * @param ctx Context to query
         * @param entries Output: Number of currently cached paths (can be NULL)
         * @param capacity Output: Maximum cache capacity (can be NULL)
         * @param hit_rate Output: Cache hit rate percentage 0-100 (can be NULL)
         */
        void get_cache_stats(CacheContext* ctx, uint32_t* entries, uint32_t* capacity, uint32_t* hit_rate);
    } // namespace cache
} // namespace pathfinder

#endif