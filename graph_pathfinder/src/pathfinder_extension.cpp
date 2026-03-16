
#include <cstdint>
#include <cstdio>
#include <pathfinder_extension.h>
#include <dmsdk/dlib/hashtable.h>
#include "dmsdk/dlib/log.h"
#include "dmsdk/dlib/time.h"
#include "dmsdk/gameobject/gameobject.h"
#include "pathfinder_constants.h"
#include "pathfinder_path.h"
#include "pathfinder_smooth.h"

// #include "pathfinder_cache.h"
// #include "pathfinder_distance_cache.h"
// #include "pathfinder_spatial_index.h"
#include <pathfinder_navmesh.h>
namespace pathfinder
{
    namespace extension
    {
        //==========================================================
        // Smooth
        //==========================================================
        typedef struct SmoothConfig
        {
            pathfinder::PathSmoothStyle  m_PathSmoothStyle;
            pathfinder::PathSmoothConfig m_PathSmoothConfig;
        } SmoothConfig;

        const static uint8_t               MAX_SMOOTH_CONFIG = 64;
        static dmHashTable16<SmoothConfig> m_SmoothConfigs;
        static uint32_t                    m_SmoothId = 0;

        //==========================================================
        // Gameobjects
        //==========================================================
        enum GameobjectState
        {
            RUNNING = 0,
            PAUSED = 1,
        };

        typedef struct Gameobject
        {
            int32_t                 m_NodeId;
            dmVMath::Point3         m_Position;
            dmGameObject::HInstance m_GameObjectInstance;
            bool                    m_UseWorldPosition;
            GameobjectState         m_GameobjectState;
        } Gameobject;

        static dmHashTable32<Gameobject> m_Gameobjects;

        //==========================================================
        // Navmeshs
        //==========================================================
        static dmHashTable16<pathfinder::navmesh::NavMeshContext*> m_NavmeshContext;
        static uint8_t                                             m_NavmeshId = 0;

        //==========================================================
        // Update
        //==========================================================
        static uint8_t  m_UpdateFrequency;
        static float    m_MaxTimeStep;
        static uint64_t m_PreviousFrameTime;
        static float    m_AccumFrameTime;
        static bool     m_UpdateLoopState = true;

        // From Defold source
        // https://github.com/defold/defold/blob/cdaa870389ca00062bfc03bcda8f4fb34e93124a/engine/engine/src/engine.cpp#L1860
        static void calc_timestep(float& step_dt, uint32_t& num_steps)
        {
            uint64_t time = dmTime::GetMonotonicTime();
            uint64_t frame_time = time - m_PreviousFrameTime;
            m_PreviousFrameTime = time;

            float frame_dt = (float)(frame_time / 1000000.0);

            // Never allow for large hitches
            if (frame_dt > m_MaxTimeStep)
            {
                frame_dt = m_MaxTimeStep;
            }

            // Variable frame rate
            if (m_UpdateFrequency == 0)
            {
                step_dt = frame_dt;
                num_steps = 1;
                return;
            }

            // Fixed frame rate
            float fixed_dt = 1.0f / (float)m_UpdateFrequency;

            // We don't allow having a higher framerate than the actual variable frame
            // rate since the update+render is currently coupled together and also Flip()
            // would be called more than once. E.g. if the fixed_dt == 1/120 and the
            // frame_dt == 1/60
            if (fixed_dt < frame_dt)
            {
                fixed_dt = frame_dt;
            }

            m_AccumFrameTime += frame_dt;

            float num_steps_f = m_AccumFrameTime / fixed_dt;

            num_steps = (uint32_t)num_steps_f;
            step_dt = fixed_dt;

            m_AccumFrameTime = m_AccumFrameTime - num_steps * fixed_dt;
        }

        static inline void gameobject_iterate_callback(void* /*context*/, const uint32_t* /*key*/, Gameobject* gameobject)
        {
            if (gameobject->m_GameobjectState == GameobjectState::PAUSED)
            {
                return;
            }

            if (gameobject->m_UseWorldPosition)
            {
                gameobject->m_Position = dmGameObject::GetWorldPosition(gameobject->m_GameObjectInstance);
            }
            else
            {
                gameobject->m_Position = dmGameObject::GetPosition(gameobject->m_GameObjectInstance);
            }

            pathfinder::path::move_node(gameobject->m_NodeId, Vec2(gameobject->m_Position.getX(), gameobject->m_Position.getY()));
        }

        //==========================================================
        // OPs
        //==========================================================

        //
        void init()
        {
            m_SmoothConfigs.SetCapacity(MAX_SMOOTH_CONFIG);
        }

        void shutdown()
        {
            m_Gameobjects.Clear();
            m_SmoothConfigs.Clear();
            m_SmoothId = 0;
        }

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
                             uint32_t& spatial_index_max_edges_per_cell)
        {
            pathfinder::path::get_cache_stats(&path_cache_entries, &path_cache_capacity, &path_cache_hit_rate);

            pathfinder::path::get_distance_cache_stats(&dist_cache_size, &dist_cache_hits, &dist_cache_misses, &dist_cache_hit_rate);

            pathfinder::path::get_spatial_index_stats(&spatial_index_cell_count, &spatial_index_edge_count, &spatial_index_avg_edges_per_cell, &spatial_index_max_edges_per_cell);
        }

        //==========================================================
        // Gameobjects
        //==========================================================

        void set_gameobject_capacity(uint32_t gameobject_capacity)
        {
            m_Gameobjects.SetCapacity(gameobject_capacity);
        }

        void add_gameobject_node(uint32_t node_id, dmGameObject::HInstance instance, dmVMath::Point3 position, bool use_world_position)
        {
            if (m_Gameobjects.Full())
            {
                dmLogError("max_gameobject_nodes not defined on init or it is full. Size: %u", m_Gameobjects.Size());
                return;
            }
            Gameobject gameobject;
            gameobject.m_NodeId = node_id;
            gameobject.m_Position = position;
            gameobject.m_GameObjectInstance = instance;
            gameobject.m_GameobjectState = GameobjectState::RUNNING;
            gameobject.m_UseWorldPosition = use_world_position;

            m_Gameobjects.Put(node_id, gameobject);
        }

        void remove_gameobject_node(uint32_t node_id)
        {
            m_Gameobjects.Erase(node_id);
        }

        void pause_gameobject_node(uint32_t node_id)
        {
            Gameobject* gameobject = m_Gameobjects.Get(node_id);
            if (gameobject == 0x0)
            {
                dmLogWarning("Cannot pause gameobject node %u: not found", node_id);
                return;
            }
            gameobject->m_GameobjectState = GameobjectState::PAUSED;
        }

        void resume_gameobject_node(uint32_t node_id)
        {
            Gameobject* gameobject = m_Gameobjects.Get(node_id);
            if (gameobject == 0x0)
            {
                dmLogWarning("Cannot resume gameobject node %u: not found", node_id);
                return;
            }
            gameobject->m_GameobjectState = GameobjectState::RUNNING;
        }

        //==========================================================
        // Update
        //==========================================================

        void set_update_state(bool state)
        {
            m_UpdateLoopState = state;
        }

        void set_max_time_step(float max_time_step)
        {
            m_MaxTimeStep = max_time_step;
        };

        void set_update_frequency(uint8_t update_frequency)
        {
            m_UpdateFrequency = update_frequency;
        }

        void update()
        {
            // If paused or not set
            if (!m_UpdateLoopState || m_Gameobjects.Empty())
            {
                return;
            }

            float    step_dt;   // The dt for each step (the game frame)
            uint32_t num_steps; // Number of times to loop over the StepFrame function

            calc_timestep(step_dt, num_steps);
            for (uint32_t i = 0; i < num_steps; ++i)
            {
                m_Gameobjects.Iterate(gameobject_iterate_callback, (void*)0x0);
            }
        }

        //==========================================================
        // Smooth
        //==========================================================

        uint32_t add_smooth_config(uint32_t path_style, const pathfinder::PathSmoothConfig path_smooth_config)
        {
            if (m_SmoothConfigs.Full())
            {
                dmLogError("Smooth config full. Size: %u", MAX_SMOOTH_CONFIG);
                return 0;
            }

            SmoothConfig smooth_config;
            smooth_config.m_PathSmoothStyle = (pathfinder::PathSmoothStyle)path_style;
            smooth_config.m_PathSmoothConfig = path_smooth_config;

            m_SmoothId++;
            m_SmoothConfigs.Put(m_SmoothId, smooth_config);
            return m_SmoothId;
        }

        void update_smooth_config(uint32_t smooth_id, uint32_t path_style, const pathfinder::PathSmoothConfig path_smooth_config)
        {
            SmoothConfig* smooth_config = m_SmoothConfigs.Get(smooth_id);
            if (smooth_config == 0x0)
            {
                dmLogError("Invalid smooth_id %u: config not found", smooth_id);
                return;
            }

            smooth_config->m_PathSmoothStyle = (pathfinder::PathSmoothStyle)path_style;
            smooth_config->m_PathSmoothConfig = path_smooth_config;
        }

        uint32_t get_smooth_sample_segment(uint32_t smooth_id)
        {
            SmoothConfig* smooth_config = m_SmoothConfigs.Get(smooth_id);
            if (smooth_config == 0x0)
            {
                dmLogError("Invalid smooth_id %u: config not found", smooth_id);
                return 0;
            }
            return smooth_config->m_PathSmoothConfig.m_SampleSegment;
        }

        void smooth_path(uint32_t smooth_id, dmArray<uint32_t>& path, dmArray<Vec2>& smoothed_path)
        {
            SmoothConfig* smooth_config = m_SmoothConfigs.Get(smooth_id);
            if (smooth_config == 0x0)
            {
                dmLogError("Invalid smooth_id %u: config not found", smooth_id);
                return;
            }

            switch (smooth_config->m_PathSmoothStyle)
            {
                case NONE:
                    break;
                case CATMULL_ROM:
                    pathfinder::smooth::catmull_rom(path, smoothed_path, smooth_config->m_PathSmoothConfig.m_SampleSegment);
                    break;
                case BEZIER_CUBIC:
                    pathfinder::smooth::bezier_cubic(path, smoothed_path, smooth_config->m_PathSmoothConfig.m_SampleSegment, smooth_config->m_PathSmoothConfig.m_ControlPointOffset);
                    break;
                case BEZIER_QUADRATIC:
                    pathfinder::smooth::bezier_quadratic(path, smoothed_path, smooth_config->m_PathSmoothConfig.m_SampleSegment, smooth_config->m_PathSmoothConfig.m_CurveRadius);
                    break;
                case BEZIER_ADAPTIVE:
                    pathfinder::smooth::bezier_adaptive(path, smoothed_path, smooth_config->m_PathSmoothConfig.m_SampleSegment, smooth_config->m_PathSmoothConfig.m_BezierAdaptiveTightness, smooth_config->m_PathSmoothConfig.m_BezierAdaptiveRoundness, smooth_config->m_PathSmoothConfig.m_BezierAdaptiveMaxCornerDist);
                    break;
                case CIRCULAR_ARC:
                    pathfinder::smooth::circular_arc(path, smoothed_path, smooth_config->m_PathSmoothConfig.m_SampleSegment, smooth_config->m_PathSmoothConfig.m_ArcRadius);
                    break;
            }
        }

        void smooth_path_waypoint(uint32_t smooth_id, dmArray<Vec2>& waypoints, dmArray<Vec2>& smoothed_path)
        {
            SmoothConfig* smooth_config = m_SmoothConfigs.Get(smooth_id);
            if (smooth_config == 0x0)
            {
                dmLogError("Invalid smooth_id %u: config not found", smooth_id);
                return;
            }

            switch (smooth_config->m_PathSmoothStyle)
            {
                case NONE:
                    break;
                case CATMULL_ROM:
                    pathfinder::smooth::catmull_rom_waypoints(waypoints, smoothed_path, smooth_config->m_PathSmoothConfig.m_SampleSegment);
                    break;
                case BEZIER_CUBIC:
                    pathfinder::smooth::bezier_cubic_waypoints(waypoints, smoothed_path, smooth_config->m_PathSmoothConfig.m_SampleSegment, smooth_config->m_PathSmoothConfig.m_ControlPointOffset);
                    break;
                case BEZIER_QUADRATIC:
                    pathfinder::smooth::bezier_quadratic_waypoints(waypoints, smoothed_path, smooth_config->m_PathSmoothConfig.m_SampleSegment, smooth_config->m_PathSmoothConfig.m_CurveRadius);
                    break;
                case BEZIER_ADAPTIVE:
                    pathfinder::smooth::bezier_adaptive_waypoints(waypoints, smoothed_path, smooth_config->m_PathSmoothConfig.m_SampleSegment, smooth_config->m_PathSmoothConfig.m_BezierAdaptiveTightness, smooth_config->m_PathSmoothConfig.m_BezierAdaptiveRoundness, smooth_config->m_PathSmoothConfig.m_BezierAdaptiveMaxCornerDist);
                    break;
                case CIRCULAR_ARC:
                    pathfinder::smooth::circular_arc_waypoints(waypoints, smoothed_path, smooth_config->m_PathSmoothConfig.m_SampleSegment, smooth_config->m_PathSmoothConfig.m_ArcRadius);
                    break;
            }
        }

        uint8_t navmesh_init(pathfinder::navmesh::NavMeshContext* ctx)
        {
            if (m_NavmeshContext.Full())
            {
                m_NavmeshContext.SetCapacity(m_NavmeshContext.Size() + 1);
            }
            m_NavmeshId++;
            m_NavmeshContext.Put(m_NavmeshId, ctx);
            return m_NavmeshId;
        }

        static inline pathfinder::navmesh::NavMeshContext* get_navmesh_ctx(uint8_t id)
        {
            pathfinder::navmesh::NavMeshContext** ctx = m_NavmeshContext.Get(id);
            if (!ctx)
                return 0;
            return *ctx;
        }

        static inline void navmesh_iterate_callback(void* /*context*/, const uint16_t* /*key*/, pathfinder::navmesh::NavMeshContext** ctx)
        {
            pathfinder::navmesh::destroy_context(*ctx);
        }

        void navmesh_shutdown()
        {
            m_NavmeshContext.Iterate(navmesh_iterate_callback, (void*)0x0);
            m_NavmeshContext.Clear();
            m_NavmeshId = 0;
        }

        void navmesh_remove(uint8_t navmesh_id)
        {
            pathfinder::navmesh::NavMeshContext* ctx = get_navmesh_ctx(navmesh_id);
            if (!ctx)
                return;

            pathfinder::navmesh::destroy_context(ctx);
            m_NavmeshContext.Erase(navmesh_id);
        }

        void navmesh_set_buffer(uint8_t navmesh_id, dmBuffer::HBuffer& buffer)
        {
            pathfinder::navmesh::NavMeshContext* ctx = get_navmesh_ctx(navmesh_id);
            if (!ctx)
                return;

            void*            data = 0;
            uint32_t         count = 0;
            uint32_t         components = 0;
            uint32_t         stride = 0;
            dmBuffer::Result buffer_result = dmBuffer::GetStream(buffer, dmHashString64("position"), &data, &count, &components, &stride);

            if (buffer_result != dmBuffer::RESULT_OK)
            {
                dmLogError("No position stream");
                return;
            }

            dmLogInfo("Position stream: vertices=%u components=%u stride=%u\n", count, components, stride);

            if (count % 3 != 0)
            {
                dmLogError("Vertex count must be multiple of 3 for triangles");
                return;
            }

            uint32_t tri_count = count / 3;
            if (tri_count == 0)
            {
                dmLogWarning("No triangles in buffer");
                return;
            }

            uint32_t vertex_stride = stride;
            if (stride < components * sizeof(float))
            {
                vertex_stride = components * sizeof(float);
            }

            uint32_t               z_component = (components == 3) ? 2 : 1;
            uint8_t*               bytes = (uint8_t*)data;
            pathfinder::PathStatus status;

            for (uint32_t t = 0; t < tri_count; ++t)
            {
                pathfinder::Vec2 vertices[3];

                for (uint32_t v = 0; v < 3; ++v)
                {
                    float* floats = (float*)(bytes + ((t * 3 + v) * vertex_stride));
                    vertices[v] = pathfinder::Vec2(floats[0], floats[z_component]);
                    //    dmLogInfo("t: %u - x %f -  y: % f", t, vertices[v].x, vertices[v].y);
                }

                // not doing anything with cell_id yet
                uint32_t cell_id = pathfinder::navmesh::add_cell(ctx, vertices, 3, &status);
                if (status != pathfinder::SUCCESS)
                {
                    dmLogError("Failed to add cell %u (status: %d)", t, status);
                    return;
                }
            }
            pathfinder::navmesh::build_adjacency(ctx);

            dmLogInfo("Successfully built navmesh with %u triangles", tri_count);
        }

        void navmesh_find_path(uint8_t          navmesh_id,
                               uint32_t*        path_length,
                               pathfinder::Vec2 start_position,
                               pathfinder::Vec2 goal_position,
                               dmArray<Vec2>*   smooth_path,
                               uint32_t         max_path,
                               float            agent_radius,
                               bool             enable_fallback,
                               PathStatus*      status)
        {
            pathfinder::navmesh::NavMeshContext* ctx = get_navmesh_ctx(navmesh_id);
            if (!ctx)
                return;

            *path_length = pathfinder::navmesh::find_path_from_positions(ctx,
                                                                         start_position,
                                                                         goal_position,
                                                                         smooth_path,
                                                                         max_path,
                                                                         agent_radius,
                                                                         enable_fallback,
                                                                         status);
        }

        void navmesh_cell_at_position(uint8_t navmesh_id, pathfinder::Vec2 position, uint32_t* cell_id, pathfinder::Vec2* center)
        {
            pathfinder::navmesh::NavMeshContext* ctx = get_navmesh_ctx(navmesh_id);
            if (!ctx)
                return;

            *cell_id = pathfinder::navmesh::find_cell_at_position(ctx, position, false);
            *center = pathfinder::navmesh::get_cell_center(ctx, *cell_id);
        }

        navmesh::NavMeshSpatialIndex* navmesh_get_spatial_index(uint8_t navmesh_id)
        {
            pathfinder::navmesh::NavMeshContext* ctx = get_navmesh_ctx(navmesh_id);
            if (!ctx)
                return 0;

            return pathfinder::navmesh::get_spatial_index(ctx);
        }

        void navmesh_set_funnel(uint8_t navmesh_id, float portal_vertex_tolerance, float portal_collapse_threshold, float waypoint_duplicate_tolerance)
        {
            pathfinder::navmesh::NavMeshContext* ctx = get_navmesh_ctx(navmesh_id);
            if (!ctx)
                return;

            pathfinder::navmesh::funnel_init(ctx, portal_vertex_tolerance, portal_collapse_threshold, waypoint_duplicate_tolerance);
        }

        void navmesh_get_stats(uint8_t   navmesh_id,
                               uint32_t& cache_entries,
                               uint32_t& cache_capacity,
                               uint32_t& cache_hit_rate,
                               uint32_t& dist_cache_size,
                               uint32_t& dist_cache_hits,
                               uint32_t& dist_cache_misses,
                               uint32_t& dist_cache_hit_rate)
        {
            pathfinder::navmesh::NavMeshContext* ctx = get_navmesh_ctx(navmesh_id);
            if (!ctx)
                return;

            // Path cache statistics
            pathfinder::cache::CacheContext* cache_ctx = pathfinder::navmesh::get_cache_context(ctx);

            if (cache_ctx)
            {
                pathfinder::cache::get_cache_stats(cache_ctx, &cache_entries, &cache_capacity, &cache_hit_rate);
            }

            // Distance cache statistics
            pathfinder::distance_cache::DistanceCacheContext* dist_cache_ctx = pathfinder::navmesh::get_distance_cache_context(ctx);
            if (dist_cache_ctx)
            {
                pathfinder::distance_cache::get_stats(dist_cache_ctx, &dist_cache_size, &dist_cache_hits, &dist_cache_misses, &dist_cache_hit_rate);
            }
        }

    } // namespace extension
} // namespace pathfinder