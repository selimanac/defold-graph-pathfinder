
#include <pathfinder_extension.h>
#include <dmsdk/dlib/hashtable.h>
#include "pathfinder_constants.h"
#include "pathfinder_smooth.h"

namespace pathfinder
{
    namespace extension
    {

        enum GameobjectState
        {
            RUNNING = 0,
            PAUSED = 1,
        };

        typedef struct SmoothConfig
        {
            pathfinder::PathSmoothStyle       m_PathSmoothStyle;
            navigation::AgentPathSmoothConfig m_PathSmoothConfig;
        } SmoothConfig;

        typedef struct Gameobject
        {
            int32_t                 m_NodeId;
            dmVMath::Point3         m_Position;
            dmGameObject::HInstance m_GameObjectInstance;
            bool                    m_UseWorldPosition;
            GameobjectState         m_GameobjectState;
        } Gameobject;

        static dmHashTable16<SmoothConfig> m_SmoothConfigs;
        static dmHashTable32<Gameobject>   m_Gameobjects;
        static uint32_t                    m_SmoothId = 0;

        //
        void init()
        {
            m_SmoothConfigs.SetCapacity(64); // TODO Check if hit capacity
        }

        void set_gameobject_capacity(uint32_t gameobject_capacity)
        {
            m_Gameobjects.SetCapacity(gameobject_capacity);
        }

        void add_gameobject_node(uint32_t node_id, dmGameObject::HInstance instance, dmVMath::Point3 position, bool use_world_position)
        {
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