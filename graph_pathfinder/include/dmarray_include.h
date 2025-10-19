#ifndef DMARRAY_INCLUDE_H
#define DMARRAY_INCLUDE_H

/**
 * @file dmarray_include.h
 * @brief Conditional include wrapper for dmArray
 * 
 * This header provides a unified way to include dmArray regardless of whether
 * we're using the local copy or the Defold SDK version.
 * 
 * When ENABLE_DEFOLD_SDK=ON:
 *   - Uses Defold SDK's dmArray from <dmsdk/dlib/array.h>
 *   - Ensures compatibility with other Defold SDK components
 * 
 * When ENABLE_DEFOLD_SDK=OFF (default):
 *   - Uses local copy from "dmarray.h"
 *   - Standalone operation without SDK dependency
 * 
 * This prevents header conflicts when building with Defold SDK integration.
 */

#ifdef USE_DEFOLD_SDK_HEADERS
    // Use Defold SDK's official dmArray header
    #include <dmsdk/dlib/array.h>
#else
    // Use local copy of dmArray
    #include "dmarray.h"
#endif

#endif // DMARRAY_INCLUDE_H
