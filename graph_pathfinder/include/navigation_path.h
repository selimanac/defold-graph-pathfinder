#ifndef NAVIGATION_PATH_H
#define NAVIGATION_PATH_H

#include "navigation_types.h"

namespace navigation
{
    namespace path
    {

        //     static void     smooth_agent_path(Agent* agent);

        void     init(uint32_t max_path_length, uint32_t max_samples_per_segment);

        uint32_t find_agent_path(Agent* agent);
    } // namespace path
} // namespace navigation

#endif