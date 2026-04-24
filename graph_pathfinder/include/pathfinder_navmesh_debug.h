/**
 * @file pathfinder_navmesh_debug.h
 * @brief Shared debug configuration for NavMesh subsystems
 *
 * This header provides centralized debug macros for all NavMesh-related
 * modules (navmesh, spatial, astar, funnel). It eliminates duplicate debug
 * definitions across multiple files.
 *
 * Usage:
 * 1. Include this header in NavMesh implementation files
 * 2. Use NAVMESH_LOG(flag, ...) for debug output, passing the context's
 *    m_DebugMode flag (or a local bool debug parameter) explicitly
 */

#ifndef PATHFINDER_NAVMESH_DEBUG_H
#define PATHFINDER_NAVMESH_DEBUG_H

// Enable for debugging NavMesh operations (compile-time control)
// Set to 0 to completely remove debug code (zero runtime overhead)
#define NAVMESH_DEBUG 1

#if NAVMESH_DEBUG
#include <stdio.h>

// Centralized debug logging macro
// Usage: NAVMESH_LOG(debug_flag, "format string", args...)
// Pass the context's m_DebugMode (or a local bool debug parameter) explicitly.
// Example: NAVMESH_LOG(ctx->m_DebugMode, "Initialized: max_cells=%u", max_cells)
#define NAVMESH_LOG(flag, format, ...) \
    do \
    { \
        if (flag) \
            printf("[NavMesh] " format "\n", ##__VA_ARGS__); \
    } while (0)

#else
// Debug disabled at compile time - all macros become no-ops
#define NAVMESH_LOG(flag, format, ...) ((void)0)
#endif // NAVMESH_DEBUG

#endif // PATHFINDER_NAVMESH_DEBUG_H
