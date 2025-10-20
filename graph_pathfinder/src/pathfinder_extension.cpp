#include <cstdio>
#include <pathfinder_extension.h>
#include <dmsdk/dlib/hashtable.h>
#include "pathfinder_constants.h"
#include "pathfinder_smooth.h"

namespace pathfinder
{
    namespace extension
    {
        struct SmoothConfig
        {
            pathfinder::PathSmoothStyle       m_PathSmoothStyle;
            navigation::AgentPathSmoothConfig m_PathSmoothConfig;
        };

        static dmHashTable16<SmoothConfig> m_SmoothConfig;
        static uint32_t                    m_SmoothId = 0;

        //
        void init()
        {
            m_SmoothConfig.SetCapacity(64);
        }
        uint32_t add_smooth_config(uint32_t path_style, navigation::AgentPathSmoothConfig path_smooth_config)
        {
            SmoothConfig smooth_config;
            smooth_config.m_PathSmoothStyle = (pathfinder::PathSmoothStyle)path_style;
            smooth_config.m_PathSmoothConfig = path_smooth_config;

            m_SmoothId++;
            m_SmoothConfig.Put(m_SmoothId, smooth_config);
            return m_SmoothId;
        }

        uint32_t get_smooth_sample_segment(uint32_t smooth_id)
        {
            SmoothConfig* smooth_config = m_SmoothConfig.Get(smooth_id);
            return smooth_config->m_PathSmoothConfig.m_SampleSegment;
        }
        void smooth_path_waypoint(uint32_t smooth_id, dmArray<Vec2>& waypoints, dmArray<Vec2>& smoothed_path)
        {
            SmoothConfig* smooth_config = m_SmoothConfig.Get(smooth_id);

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
            SmoothConfig* smooth_config = m_SmoothConfig.Get(smooth_id);

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
            m_SmoothConfig.Clear();
            m_SmoothId = 0;
        }
    } // namespace extension
} // namespace pathfinder