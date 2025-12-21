/**
 * @file pathfinder_navmesh_debug.h
 * @brief Shared debug configuration for NavMesh subsystems
 *
 * This header provides centralized debug macros and flag management for all
 * NavMesh-related modules (navmesh, spatial, astar, funnel). It eliminates
 * duplicate debug definitions across multiple files.
 *
 * Usage:
 * 1. Include this header in NavMesh implementation files
 * 2. Use NAVMESH_LOG() macro for debug output
 * 3. Set debug mode via navmesh::init() or spatial::set_debug_mode()
 */

#ifndef PATHFINDER_NAVMESH_DEBUG_H
#define PATHFINDER_NAVMESH_DEBUG_H

// Enable for debugging NavMesh operations (compile-time control)
// Set to 0 to completely remove debug code (zero runtime overhead)
#define NAVMESH_DEBUG 1

#if NAVMESH_DEBUG
#include <stdio.h>

// Runtime debug flags (only active when NAVMESH_DEBUG is enabled)
// These are defined in pathfinder_navmesh.cpp and can be set via init()
namespace pathfinder
{
    namespace navmesh
    {
        extern bool g_DebugMode; // Main NavMesh debug flag

        namespace spatial
        {
            extern bool g_SpatialDebugMode; // Spatial index debug flag
        }
    } // namespace navmesh
} // namespace pathfinder

// Centralized debug logging macro
// Usage: NAVMESH_LOG(debug_flag, "format string", args...)
// Example: NAVMESH_LOG(pathfinder::navmesh::g_DebugMode, "Initialized: max_cells=%u", max_cells)
// Note: Typically use the convenience macros NAVMESH_LOG_MAIN() or NAVMESH_LOG_SPATIAL() instead
#define NAVMESH_LOG(flag, format, ...) \
    do \
    { \
        if (flag) \
            printf("[NavMesh] " format "\n", ##__VA_ARGS__); \
    } while (0)

// Convenience macros for specific subsystems
#define NAVMESH_LOG_MAIN(format, ...) NAVMESH_LOG(pathfinder::navmesh::g_DebugMode, format, ##__VA_ARGS__)
#define NAVMESH_LOG_SPATIAL(format, ...) NAVMESH_LOG(pathfinder::navmesh::spatial::g_SpatialDebugMode, format, ##__VA_ARGS__)

#else
// Debug disabled at compile time - all macros become no-ops
#define NAVMESH_LOG(flag, format, ...) ((void)0)
#define NAVMESH_LOG_MAIN(format, ...) ((void)0)
#define NAVMESH_LOG_SPATIAL(format, ...) ((void)0)
#endif // NAVMESH_DEBUG

#endif // PATHFINDER_NAVMESH_DEBUG_H
