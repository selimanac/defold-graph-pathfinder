#ifndef PATHFINDER_EXTENSION_H
#define PATHFINDER_EXTENSION_H

#include <dmsdk/dlib/array.h>
#include "navigation_types.h"

namespace pathfinder
{
    namespace extension
    {

        //
        void     init();
        uint32_t add_smooth_config(uint32_t path_style, navigation::AgentPathSmoothConfig path_smooth_config);
        void     smooth_path(uint32_t smooth_id, dmArray<uint32_t>& path, dmArray<Vec2>& smoothed_path);

        uint32_t get_smooth_sample_segment(uint32_t smooth_id);
        void     shutdown();

    } // namespace extension
} // namespace pathfinder
#endif