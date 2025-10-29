#ifndef PATHFINDER_TYPES_H
#define PATHFINDER_TYPES_H

#include <cstdint>

/**
 * @file pathfinder_types.h
 * @brief Core data types for the pathfinding system
 *
 * This header defines the fundamental data structures used throughout the
 * pathfinding engine. All types are POD (Plain Old Data) structures for
 * maximum performance and cache-friendliness.
 *
 * Design Principles:
 * - Simple C-style structs (no virtual functions or inheritance)
 * - Flat memory layout for cache efficiency
 * - Minimal size for array packing (Vec2: 8 bytes, Edge: 8 bytes, Node: 16 bytes)
 * - Direct member access (no getters/setters)
 *
 * Usage Pattern:
 * - Vec2: 2D positions, velocities, and directions
 * - Edge: Graph connectivity with weighted costs
 * - Node: Spatial graph nodes with version tracking
 *
 * Thread Safety: Types themselves are thread-safe (POD), but concurrent
 * access to shared instances requires external synchronization.
 */

namespace pathfinder
{
    /**
     * @brief 2D Vector for positions, velocities, and directions
     *
     * General-purpose 2D vector type used throughout the engine for:
     * - Node positions in 2D space
     * - Waypoint locations for path smoothing
     * - Agent velocities and accelerations (navigation system)
     * - Direction vectors and offsets
     *
     * Coordinate System:
     * - Arbitrary coordinate system (game-dependent)
     * - Typically: +X right, +Y up (or +Y down for screen space)
     * - Units: Game-defined (pixels, meters, tiles, etc.)
     *
     * Memory Layout: 8 bytes (2 * 4-byte floats)
     * Alignment: 4-byte aligned (float alignment)
     *
     * Operations:
     * Use pathfinder::math namespace for vector operations:
     * - distance(), length(), normalize()
     * - add(), subtract(), scale()
     * - dot product, projection, interpolation
     */
    typedef struct Vec2
    {
        float x, y; ///< X and Y coordinates in 2D space

        /**
         * @brief Default constructor - initializes to zero vector (0, 0)
         */
        Vec2()
            : x(0.0f)
            , y(0.0f)
        {
        }

        /**
         * @brief Parameterized constructor
         * @param x_ X coordinate value
         * @param y_ Y coordinate value
         */
        Vec2(float x_, float y_)
            : x(x_)
            , y(y_)
        {
        }
    } Vec2;

    /**
     * @brief Directed edge in the pathfinding graph
     *
     * Represents a one-way connection from a source node to a destination node
     * with an associated traversal cost. Edges are stored in a flat array indexed
     * per-node for cache-efficient iteration during pathfinding.
     *
     * Usage:
     * - Stored in pathfinder::path::m_Edges flat array
     * - Indexed per-node via m_EdgesIndex and m_EdgeCount arrays
     * - Cost typically = Euclidean distance, but can be weighted for terrain
     *
     * Cost Interpretation:
     * - Lower cost = preferred path (A* minimizes total cost)
     * - Typical: cost = distance between nodes
     * - Custom: cost = distance * terrain_multiplier (mud: 2.0x, water: 5.0x, etc.)
     *
     * Bidirectional Edges:
     * - Stored as two separate Edge instances (A→B and B→A)
     * - Created automatically when add_edge() called with bidirectional=true
     * - Can have asymmetric costs (uphill vs downhill)
     * - m_Bidirectional flag set to true on both edges for O(1) detection
     *
     * Memory Layout: 12 bytes (4-byte uint32_t + 4-byte float + 1-byte bool + 3 bytes padding)
     * Alignment: 4-byte aligned
     */
    typedef struct Edge
    {
        uint32_t m_To;             ///< Destination node ID (index into m_Nodes array)
        float    m_Cost;           ///< Traversal cost (typically distance, but can be weighted)
        bool     m_Bidirectional;  ///< True if reverse edge exists (eliminates O(E) has_edge() scan)
    } Edge;

    /**
     * @brief Node in the pathfinding graph with spatial position and version tracking
     *
     * Represents a single waypoint or location in the pathfinding graph. Nodes are
     * stored in a flat array with active/inactive flags for dynamic add/remove.
     *
     * Node Lifecycle:
     * 1. add_node(): Finds first inactive slot, assigns ID, sets position
     * 2. Active: Node participates in pathfinding, edges can connect to it
     * 3. remove_node(): Marks as inactive, removes edges, frees slot for reuse
     *
     * Version Tracking:
     * - m_Version increments when node position changes (move_node)
     * - Used for fine-grained cache invalidation
     * - Only paths containing this specific node are invalidated
     * - Separate from global node/edge version counters
     *
     * ID Assignment:
     * - m_Id is the index in m_Nodes array (0 to max_nodes-1)
     * - Stable until node is removed (not reassigned on remove)
     * - Removed node IDs are reused by next add_node() call
     *
     * Memory Layout: 16 bytes (4-byte ID + 8-byte Vec2 + 4-byte version)
     * Alignment: 4-byte aligned
     *
     * Optimization Notes:
     * - Flat array storage for cache-friendly iteration
     * - Per-node active flags stored separately in m_NodeActive array
     * - Position and version together for cache locality during pathfinding
     */
    typedef struct Node
    {
        uint32_t m_Id;       ///< Node ID (index in m_Nodes array, stable until removed)
        Vec2     m_Position; ///< 2D spatial position in world coordinates
        uint32_t m_Version;  ///< Per-node version counter (increments on position change)
    } Node;

    /**
     * @brief Extended edge information including bidirectionality status
     *
     * This structure provides complete edge information for a specific node,
     * including source, destination, cost, and whether the edge is part of
     * a bidirectional connection.
     *
     * Usage:
     * - Returned by get_node_edges() helper function
     * - m_From: Always equals the queried node_id
     * - m_To: Destination node ID
     * - m_Cost: Edge traversal cost (same as in Edge struct)
     * - m_Bidirectional: true if reverse edge (m_To -> m_From) also exists
     *
     * Bidirectionality Detection:
     * - Determined by checking if reverse edge exists at query time
     * - Not stored persistently (computed on demand)
     * - Two edges A->B and B->A may have different costs
     * - m_Bidirectional is true if both directions exist (regardless of cost)
     *
     * Memory Layout: 16 bytes (3 * 4-byte fields + 1-byte bool + 3 bytes padding)
     * Alignment: 4-byte aligned
     */
    typedef struct EdgeInfo
    {
        uint32_t m_From;          ///< Source node ID (same as queried node_id)
        uint32_t m_To;            ///< Destination node ID
        float    m_Cost;          ///< Edge traversal cost
        bool     m_Bidirectional; ///< True if reverse edge exists (m_To -> m_From)
    } EdgeInfo;

} // namespace pathfinder
#endif