#ifndef PATHFINDER_EXTENSION_H
#define PATHFINDER_EXTENSION_H

#include <dmsdk/dlib/array.h>
#include "navigation_types.h"
#include "dmsdk/dlib/vmath.h"
#include "dmsdk/gameobject/gameobject.h"

namespace pathfinder
{
    namespace extension
    {
        // OPs
        void init();
        void shutdown();
        void get_cache_stats(uint32_t& path_cache_entries,
                             uint32_t& path_cache_capacity,
                             uint32_t& path_cache_hit_rate,
                             uint32_t& dist_cache_size,
                             uint32_t& dist_cache_hits,
                             uint32_t& dist_cache_misses,
                             uint32_t& dist_cache_hit_rate);

        // Gameobjects
        void set_gameobject_capacity(uint32_t gameobject_capacity);
        void add_gameobject_node(uint32_t node_id, dmGameObject::HInstance instance, dmVMath::Point3 position, bool use_world_position);
        void remove_gameobject_node(uint32_t node_id);
        void pause_gameobject_node(uint32_t node_id);
        void resume_gameobject_node(uint32_t node_id);

        // Update
        void set_update_state(bool state);
        void set_update_frequency(uint8_t update_frequency);
        void update();

        // Smooth
        uint32_t add_smooth_config(uint32_t path_style, const navigation::AgentPathSmoothConfig path_smooth_config);
        void     update_smooth_config(uint32_t smooth_id, uint32_t path_style, const navigation::AgentPathSmoothConfig path_smooth_config);
        uint32_t get_smooth_sample_segment(uint32_t smooth_id);
        void     smooth_path(uint32_t smooth_id, dmArray<uint32_t>& path, dmArray<Vec2>& smoothed_path);
        void     smooth_path_waypoint(uint32_t smooth_id, dmArray<Vec2>& waypoints, dmArray<Vec2>& smoothed_path);

    } // namespace extension
} // namespace pathfinder
#endif // PATHFINDER_EXTENSION_H