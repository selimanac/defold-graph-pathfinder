#ifndef PATHFINDER_HEAP_H
#define PATHFINDER_HEAP_H

#include "pathfinder_constants.h"
#include <cstdint>
#include "dmarray_include.h"

/**
 * @file pathfinder_heap.h
 * @brief Min-heap priority queue implementation for A* pathfinding with object pooling
 *
 * This module provides a binary min-heap priority queue optimized for A* pathfinding.
 * The heap stores (node_id, f_score) pairs and maintains the min-heap property where
 * the root always has the lowest f_score (best candidate for expansion).
 *
 * Key Features:
 * - Binary min-heap with array-backed storage
 * - Object pooling for reduced allocations (currently simplified implementation)
 * - Version tracking for cache invalidation
 * - Bulk operations (build, push_many) for efficiency
 * - Inline operations (push, pop, peek) for zero-overhead access
 *
 * Heap Operations:
 * - push(): Insert element, O(log n) - bubble up
 * - pop(): Extract minimum, O(log n) - bubble down
 * - peek(): View minimum, O(1)
 * - decrease_key(): Update priority, O(n) currently (linear search)
 * - build(): Heapify array, O(n) - Floyd's algorithm
 * - push_many(): Bulk insert, O(n + k log n)
 *
 * Version Tracking:
 * - Global version (node/edge) increments on graph changes
 * - Per-node versions track individual node modifications
 * - Per-heap versions snapshot graph state at heap creation
 * - Enables fine-grained cache invalidation
 *
 * Performance Characteristics:
 * - Insert: O(log n) average and worst case
 * - Extract-min: O(log n) average and worst case
 * - Peek: O(1)
 * - Build from array: O(n) via Floyd's algorithm
 * - Space: O(n) for heap array
 *
 * Usage Pattern:
 * @code
 * // Initialize pool once at startup
 * pathfinder::heap::pool_init(1000, 32);
 *
 * // Use heap for A* search
 * HeapBlock heap;
 * pathfinder::heap::init(&heap);
 * pathfinder::heap::push(&heap, start_node, 0.0f);
 *
 * while (!pathfinder::heap::is_empty(&heap)) {
 *     uint32_t current = pathfinder::heap::pop(&heap);
 *     // Process current node...
 * }
 *
 * pathfinder::heap::reset(&heap);
 * pathfinder::heap::pool_clear();
 * @endcode
 *
 * Thread Safety: Not thread-safe. Each heap instance must be used by a single thread.
 * Global version counters are not protected by mutexes.
 */

namespace pathfinder
{
    namespace heap
    {
        /**
         * @brief Node in the min-heap priority queue
         *
         * Represents a single entry in the heap, storing the node index
         * and its priority value (f-score in A* pathfinding).
         */
        typedef struct HeapNode
        {
            uint32_t m_Index;  // Index/ID of the node in the graph
            float    m_FScore; // Priority value (lower = higher priority)
        } HeapNode;

        /**
         * @brief Tracks graph structure changes for cache invalidation
         *
         * Version numbers increment when the graph structure changes,
         * allowing cached paths to detect when they're stale.
         */
        typedef struct GraphVersion
        {
            uint32_t m_NodeVersion; // Incremented when nodes move or change state
            uint32_t m_EdgeVersion; // Incremented when edges are added/removed
        } GraphVersion;

        /**
         * @brief Individual heap instance with version tracking
         *
         * Each HeapBlock represents a separate min-heap, used during
         * pathfinding operations. The version tracks when the heap was
         * allocated to detect if cached paths are still valid.
         */
        typedef struct HeapBlock
        {
            dmArray<HeapNode> m_Nodes;      // Array-backed binary min-heap (user-allocated from pool)
            GraphVersion      m_Version;    // Graph version when heap was created
            uint32_t          m_Size;       // Current number of elements in heap
            uint32_t          m_Capacity;   // Maximum capacity of heap
            uint32_t          m_PoolOffset; // Offset in pool where this block's slice starts
        } HeapBlock;

        /**
         * @brief Global pool for true memory pooling
         *
         * Pre-allocates a large buffer of HeapNodes and gives slices to HeapBlocks.
         * This enables zero-copy memory pooling with no per-heap allocations.
         */
        typedef struct HeapPool
        {
            HeapNode*    m_Nodes;    // Pre-allocated buffer for all heaps
            GraphVersion m_Version;  // Global graph version
            uint32_t     m_Size;     // Current usage (next free offset)
            uint32_t     m_Capacity; // Total capacity of pool
        } HeapPool;

        /**
         * @brief Per-node version tracking for fine-grained cache invalidation
         *
         * Allows tracking which specific nodes have changed, enabling
         * selective path cache invalidation instead of full cache clears.
         */
        typedef struct NodeVersion
        {
            uint32_t m_Version;      // Node's current version number
            bool     m_AffectsPaths; // Does this node affect any cached paths?
        } NodeVersion;

        // Global state for version tracking
        extern GraphVersion         m_CurrentVersion; // Current graph version
        extern dmArray<NodeVersion> m_NodeVersions;   // Per-node version tracking

        /**
         * @brief Initialize the heap pool system
         * @param heap_pool_size Total capacity of the pool (max concurrent heap nodes)
         * @param pool_block_size Default size for each heap block allocation
         *
         * Initializes global state and pre-allocates tracking structures.
         * Note: Current implementation doesn't actually pool memory.
         */
        void pool_init(const uint32_t heap_pool_size, const uint32_t pool_block_size);

        /**
         * @brief Clear the heap pool and reset version tracking
         *
         * Releases all memory and resets version counters. Called during shutdown.
         */
        void pool_clear();

        /**
         * @brief Initialize a heap block for use
         * @param heap Heap block to initialize
         *
         * Allocates memory for the heap block from the pool (or directly).
         * If allocation fails, sets heap capacity to 0.
         */
        void init(HeapBlock* heap);

        /**
         * @brief Reset a heap block and return it to the pool
         * @param heap Heap block to reset
         *
         * WARNING: Current implementation doesn't actually return memory to pool.
         * It only updates bookkeeping counters.
         */
        void reset(HeapBlock* heap);

        /**
         * @brief Reset version tracking for a specific node
         * @param node_id Node ID to reset
         *
         * Clears the m_AffectsPaths flag for the given node, indicating
         * no cached paths depend on this node anymore.
         */
        void reset_node_version(uint32_t node_id);

        /**
         * @brief Swap two elements in the heap
         * @param heap Heap block
         * @param i Index of first element
         * @param j Index of second element
         *
         * Helper function for heap operations (bubble-up and bubble-down).
         * Time complexity: O(1)
         */
        static inline void swap(HeapBlock* heap, const uint32_t index_a, const uint32_t index_b)
        {
            HeapNode temp = heap->m_Nodes[index_a];
            heap->m_Nodes[index_a] = heap->m_Nodes[index_b];
            heap->m_Nodes[index_b] = temp;
        }

        /**
         * @brief Insert an element into the min-heap
         * @param heap Heap block to insert into
         * @param index Node index/ID to insert
         * @param f_score Priority value (f-score in A* pathfinding)
         *
         * Inserts a new element and maintains min-heap property using bubble-up.
         * The element with the lowest f_score is always at the root.
         *
         * Time complexity: O(log n) where n is heap size
         *
         * WARNING: Silently fails if heap is full (returns without error indication).
         * Consider returning bool or error code in production use.
         */
        static inline PathStatus push(HeapBlock* heap, const uint32_t index, const float f_score)
        {
            if (heap->m_Size >= heap->m_Capacity)
            {
                return ERROR_HEAP_FULL;
            }

            // Add new element at the end
            uint32_t current = heap->m_Size++;
            heap->m_Nodes[current] = HeapNode { index, f_score };

            // Bubble up: restore heap property by moving element up
            while (current > 0)
            {
                uint32_t parent = (current - 1) / 2; // Parent index
                if (heap->m_Nodes[parent].m_FScore <= heap->m_Nodes[current].m_FScore)
                    break; // Heap property satisfied
                swap(heap, current, parent);
                current = parent;
            }
            return SUCCESS;
        }

        static inline bool is_empty(const HeapBlock* heap)
        {
            return heap->m_Size == 0;
        }

        static inline uint32_t size(const HeapBlock* heap)
        {
            return heap->m_Size;
        }

        static inline bool is_full(const HeapBlock* heap)
        {
            return heap->m_Size >= heap->m_Capacity;
        }

        /**
         * @brief Update priority of an existing element
         * @param heap Heap block
         * @param index Node index to update
         * @param new_fscore New priority value (must be lower)
         *
         * Note: Current implementation requires linear search. For O(log n) operation,
         * maintain a separate index->position map.
         */
        static inline void decrease_key(HeapBlock* heap, uint32_t index, float new_fscore)
        {
            // Find element (linear search)
            for (uint32_t i = 0; i < heap->m_Size; i++)
            {
                if (heap->m_Nodes[i].m_Index == index)
                {
                    heap->m_Nodes[i].m_FScore = new_fscore;

                    // Bubble up if needed
                    while (i > 0)
                    {
                        uint32_t p = (i - 1) / 2;
                        if (heap->m_Nodes[p].m_FScore <= heap->m_Nodes[i].m_FScore)
                            break;
                        swap(heap, i, p);
                        i = p;
                    }
                    return;
                }
            }
        }

        /**
         * @brief Get the minimum element without removing it
         * @param heap Heap block to inspect
         * @param out_index Output parameter for node index
         * @param out_fscore Output parameter for f-score
         * @return true if heap not empty, false otherwise
         */
        static inline bool peek(HeapBlock* heap, uint32_t* out_index, float* out_fscore)
        {
            if (heap->m_Size == 0)
                return false;

            *out_index = heap->m_Nodes[0].m_Index;
            *out_fscore = heap->m_Nodes[0].m_FScore;
            return true;
        }

        /**
         * @brief Build a heap from an array of nodes (heapify)
         * @param heap Heap block to build
         * @param nodes Array of HeapNode elements to insert
         * @param count Number of nodes in the array
         * @return PathStatus SUCCESS or ERROR_HEAP_FULL if count exceeds capacity
         *
         * Builds a min-heap from an unsorted array in O(n) time, which is more
         * efficient than inserting elements one-by-one with push() (O(n log n)).
         *
         * Uses Floyd's algorithm: starts from the last non-leaf node and
         * heapifies down to the root.
         *
         * Time complexity: O(n) where n is the number of nodes
         */
        PathStatus build(HeapBlock* heap, const HeapNode* nodes, uint32_t count);

        /**
         * @brief Insert multiple elements efficiently
         * @param heap Heap block to insert into
         * @param nodes Array of HeapNode elements to insert
         * @param count Number of nodes to insert
         * @return PathStatus SUCCESS or ERROR_HEAP_FULL if any insertion fails
         *
         * More efficient than calling push() multiple times when the heap
         * is empty or nearly empty, as it can use bulk heapify.
         *
         * Time complexity: O(n + k log n) where n is initial heap size,
         * k is number of elements to insert
         */
        PathStatus push_many(HeapBlock* heap, const HeapNode* nodes, uint32_t count);

        /**
         * @brief Extract the minimum element from the heap
         * @param heap Heap block to extract from
         * @return Node index/ID with the lowest f-score, or INVALID_ID if heap is empty
         *
         * Removes and returns the root element (minimum f-score), then restores
         * the min-heap property using bubble-down (heapify-down).
         *
         * Time complexity: O(log n) where n is heap size
         *
         * Implementation details:
         * 1. Save root element (minimum)
         * 2. Move last element to root
         * 3. Bubble down: repeatedly swap with smallest child until heap property restored
         */
        static inline uint32_t pop(HeapBlock* heap)
        {
            if (heap->m_Size == 0)
            {
                return INVALID_ID; // Empty heap
            }

            // Save the minimum element (root)
            uint32_t result = heap->m_Nodes[0].m_Index;
            heap->m_Size--;

            // If heap still has elements, restore heap property
            if (heap->m_Size > 0)
            {
                // Move last element to root
                heap->m_Nodes[0] = heap->m_Nodes[heap->m_Size];

                // Bubble down: restore heap property by moving element down
                uint32_t current = 0;
                while (true)
                {
                    uint32_t left_child = 2 * current + 1;
                    uint32_t right_child = 2 * current + 2;
                    uint32_t smallest = current;

                    // Find smallest among node and its children
                    if (left_child < heap->m_Size &&
                        heap->m_Nodes[left_child].m_FScore < heap->m_Nodes[smallest].m_FScore)
                        smallest = left_child;

                    if (right_child < heap->m_Size &&
                        heap->m_Nodes[right_child].m_FScore < heap->m_Nodes[smallest].m_FScore)
                        smallest = right_child;

                    // If current node is smallest, heap property is satisfied
                    if (smallest == current)
                        break;

                    // Swap with smallest child and continue
                    swap(heap, current, smallest);
                    current = smallest;
                }
            }

            return result;
        }

    } // namespace heap
} // namespace pathfinder
#endif