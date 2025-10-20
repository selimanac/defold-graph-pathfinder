#ifndef NAVIGATION_PATHSTORAGE_H
#define NAVIGATION_PATHSTORAGE_H
#include "dmarray_include.h"
#include "pathfinder_types.h"

#include <cstdint>
namespace navigation
{
    namespace path_storage
    {

        uint32_t allocate_path_storage(uint32_t length);
        void     free_path_storage(uint32_t start, uint32_t length);

        uint32_t allocate_smoothed_path_storage(uint32_t length);
        void     free_smoothed_path_storage(uint32_t start, uint32_t length);

        void     copy_path_data(uint32_t new_start, dmArray<uint32_t>& in_path, uint32_t path_length);
        void     copy_smooth_path_data(uint32_t new_start, dmArray<pathfinder::Vec2>& in_path, uint32_t path_length);

        uint32_t get_node_id(uint32_t id); // TODO not using yet

        void     init(uint32_t raw_capacity, uint32_t smoothed_capacity);
        void     shutdown();

    } // namespace path_storage
} // namespace navigation

#endif