#ifndef DMHASHTABLE_INCLUDE_H
#define DMHASHTABLE_INCLUDE_H

/**
 * @file dmhashtable_include.h
 * @brief Conditional include wrapper for dmHashTable
 * 
 * This header provides a unified way to include dmHashTable regardless of whether
 * we're using the local copy or the Defold SDK version.
 * 
 * When ENABLE_DEFOLD_SDK=ON:
 *   - Uses Defold SDK's dmHashTable from <dmsdk/dlib/hashtable.h>
 *   - Ensures compatibility with other Defold SDK components
 * 
 * When ENABLE_DEFOLD_SDK=OFF (default):
 *   - Uses local copy from "dmhashtable.h"
 *   - Standalone operation without SDK dependency
 * 
 * This prevents header conflicts when building with Defold SDK integration.
 */

#ifdef USE_DEFOLD_SDK_HEADERS
    // Use Defold SDK's official dmHashTable header
    #include <dmsdk/dlib/hashtable.h>
#else
    // Use local copy of dmHashTable
    #include "dmhashtable.h"
#endif

#endif // DMHASHTABLE_INCLUDE_H
