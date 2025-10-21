
#include <cstdint>
#include <pathfinder_extension.h>
#include <dmsdk/dlib/hashtable.h>
#include "dmsdk/dlib/log.h"
#include "dmsdk/dlib/time.h"
#include "dmsdk/gameobject/gameobject.h"
#include "pathfinder_constants.h"
#include "pathfinder_path.h"
#include "pathfinder_smooth.h"

namespace pathfinder
{
    namespace extension
    {
        //=========================
        // Smooth
        //=========================
        typedef struct SmoothConfig
        {
            pathfinder::PathSmoothStyle       m_PathSmoothStyle;
            navigation::AgentPathSmoothConfig m_PathSmoothConfig;
        } SmoothConfig;

        const static uint8_t               MAX_SMOOTH_CONFIG = 64;
        static dmHashTable16<SmoothConfig> m_SmoothConfigs;
        static uint32_t                    m_SmoothId = 0;

        //=========================
        // Gameobjects
        //=========================
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

        //=========================
        // Upate
        //=========================
        static uint8_t  m_UpdateFrequency;
        static uint64_t m_PreviousFrameTime;
        static float    m_AccumFrameTime;
        static bool     m_UpdateLoopState = true;

        //
        void init()
        {
            m_SmoothConfigs.SetCapacity(MAX_SMOOTH_CONFIG); // TODO Check if hit capacity
        }

        // From Defold source
        // https://github.com/defold/defold/blob/cdaa870389ca00062bfc03bcda8f4fb34e93124a/engine/engine/src/engine.cpp#L1860
        static void calc_timestep(float& step_dt, uint32_t& num_steps)
        {
            uint64_t time = dmTime::GetMonotonicTime();
            uint64_t frame_time = time - m_PreviousFrameTime;
            m_PreviousFrameTime = time;

            float frame_dt = (float)(frame_time / 1000000.0);

            // Never allow for large hitches
            if (frame_dt > 0.5f)
            {
                frame_dt = 0.5f;
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

        void set_update_frequency(uint8_t update_frequency)
        {
            m_UpdateFrequency = update_frequency;
        }

        void set_update_state(bool state)
        {
            m_UpdateLoopState = state;
        }

        void pause_gameobject_node(uint32_t node_id)
        {
            Gameobject* gameobject = m_Gameobjects.Get(node_id);
            gameobject->m_GameobjectState = GameobjectState::PAUSED;
        }

        void resume_gameobject_node(uint32_t node_id)
        {
            Gameobject* gameobject = m_Gameobjects.Get(node_id);
            gameobject->m_GameobjectState = GameobjectState::RUNNING;
        }

        static inline void gameobject_iterate_callback(void*, const uint32_t* key, Gameobject* gameobject)
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

        void set_gameobject_capacity(uint32_t gameobject_capacity)
        {
            m_Gameobjects.SetCapacity(gameobject_capacity);
        }

        void remove_gameobject_node(uint32_t node_id)
        {
            m_Gameobjects.Erase(node_id);
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

        uint32_t add_smooth_config(uint32_t path_style, navigation::AgentPathSmoothConfig path_smooth_config)
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
            m_SmoothConfigs.Put(m_SmoothId, smooth_config); // TODO Check if hit capacity
            return m_SmoothId;
        }

        uint32_t get_smooth_sample_segment(uint32_t smooth_id)
        {
            SmoothConfig* smooth_config = m_SmoothConfigs.Get(smooth_id);
            return smooth_config->m_PathSmoothConfig.m_SampleSegment;
        }
        void smooth_path_waypoint(uint32_t smooth_id, dmArray<Vec2>& waypoints, dmArray<Vec2>& smoothed_path)
        {
            SmoothConfig* smooth_config = m_SmoothConfigs.Get(smooth_id);

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

        void smooth_path(uint32_t smooth_id, dmArray<uint32_t>& path, dmArray<Vec2>& smoothed_path)
        {
            SmoothConfig* smooth_config = m_SmoothConfigs.Get(smooth_id);

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

        void shutdown()
        {
            m_Gameobjects.Clear();
            m_SmoothConfigs.Clear();
            m_SmoothId = 0;
        }
    } // namespace extension
} // namespace pathfinder