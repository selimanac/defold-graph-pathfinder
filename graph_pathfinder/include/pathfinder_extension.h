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

        // Init
        void init();

        // Gameobjects
        void set_gameobject_capacity(uint32_t gameobject_capacity);

        void add_gameobject_node(uint32_t node_id, dmGameObject::HInstance instance, dmVMath::Point3 position, bool use_world_position);
        void remove_gameobject_node(uint32_t node_id);
        void set_update_gameobjects(bool state);
        void pause_gameobject_node(uint32_t node_id);
        void resume_gameobject_node(uint32_t node_id);

        // Smooth
        uint32_t add_smooth_config(uint32_t path_style, navigation::AgentPathSmoothConfig path_smooth_config);
        void     smooth_path(uint32_t smooth_id, dmArray<uint32_t>& path, dmArray<Vec2>& smoothed_path);
        void     smooth_path_waypoint(uint32_t smooth_id, dmArray<Vec2>& waypoints, dmArray<Vec2>& smoothed_path);
        uint32_t get_smooth_sample_segment(uint32_t smooth_id);

        // Shutdown
        void shutdown();

    } // namespace extension
} // namespace pathfinder
#endif