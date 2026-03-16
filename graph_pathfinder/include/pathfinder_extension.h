#ifndef PATHFINDER_EXTENSION_H
#define PATHFINDER_EXTENSION_H

#include <dmsdk/dlib/array.h>
#include "dmsdk/dlib/buffer.h"
#include "dmsdk/dlib/vmath.h"
#include "dmsdk/gameobject/gameobject.h"
#include "pathfinder_navmesh_types.h"
#include "pathfinder_types.h"

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
                             uint32_t& dist_cache_hit_rate,
                             uint32_t& spatial_index_cell_count,
                             uint32_t& spatial_index_edge_count,
                             float&    spatial_index_avg_edges_per_cell,
                             uint32_t& spatial_index_max_edges_per_cell);

        // Gameobjects
        void set_gameobject_capacity(uint32_t gameobject_capacity);
        void add_gameobject_node(uint32_t node_id, dmGameObject::HInstance instance, dmVMath::Point3 position, bool use_world_position);
        void remove_gameobject_node(uint32_t node_id);
        void pause_gameobject_node(uint32_t node_id);
        void resume_gameobject_node(uint32_t node_id);

        // Update
        void set_update_state(bool state);
        void set_update_frequency(uint8_t update_frequency);
        void set_max_time_step(float max_time_step);
        void update();

        // Smooth
        uint32_t add_smooth_config(uint32_t path_style, const pathfinder::PathSmoothConfig path_smooth_config);
        void     update_smooth_config(uint32_t smooth_id, uint32_t path_style, const pathfinder::PathSmoothConfig path_smooth_config);
        uint32_t get_smooth_sample_segment(uint32_t smooth_id);
        void     smooth_path(uint32_t smooth_id, dmArray<uint32_t>& path, dmArray<Vec2>& smoothed_path);
        void     smooth_path_waypoint(uint32_t smooth_id, dmArray<Vec2>& waypoints, dmArray<Vec2>& smoothed_path);

        // ------------------------------
        // NAVMESH
        // ------------------------------
        uint8_t                       navmesh_init(pathfinder::navmesh::NavMeshContext* ctx);
        void                          navmesh_remove(uint8_t navmesh_id);
        void                          navmesh_shutdown();
        void                          navmesh_set_buffer(uint8_t navmesh_id, dmBuffer::HBuffer& buffer);
        void                          navmesh_get_stats(uint8_t   navmesh_id,
                                                        uint32_t& cache_entries,
                                                        uint32_t& cache_capacity,
                                                        uint32_t& cache_hit_rate,
                                                        uint32_t& dist_cache_size,
                                                        uint32_t& dist_cache_hits,
                                                        uint32_t& dist_cache_misses,
                                                        uint32_t& dist_cache_hit_rate);

        void                          navmesh_find_path(uint8_t          navmesh_id,
                                                        uint32_t*        path_length,
                                                        pathfinder::Vec2 start_position,
                                                        pathfinder::Vec2 goal_position,
                                                        dmArray<Vec2>*   smooth_path,
                                                        uint32_t         max_path,
                                                        float            agent_radius,
                                                        bool             enable_fallback,
                                                        PathStatus*      status);

        void                          navmesh_cell_at_position(uint8_t navmesh_id, pathfinder::Vec2 position, uint32_t* cell_id, pathfinder::Vec2* center);

        navmesh::NavMeshSpatialIndex* navmesh_get_spatial_index(uint8_t navmesh_id);
        void                          navmesh_set_funnel(uint8_t navmesh_id, float portal_vertex_tolerance, float portal_collapse_threshold, float waypoint_duplicate_tolerance);

    } // namespace extension
} // namespace pathfinder
#endif // PATHFINDER_EXTENSION_H