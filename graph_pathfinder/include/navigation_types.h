#ifndef NAVIGATION_TYPES_H
#define NAVIGATION_TYPES_H

#include "navigation_constants.h"
#include "pathfinder_types.h"
#include <cstdint>
#include <pathfinder_constants.h>

namespace navigation
{

    struct AgentPathSmoothConfig
    {
        uint32_t m_SampleSegment;               // All
        float    m_ControlPointOffset;          // bezier_cubic
        float    m_CurveRadius;                 // bezier_quadratic
        float    m_BezierAdaptiveTightness;     // bezier_adaptive
        float    m_BezierAdaptiveRoundness;     // bezier_adaptive
        float    m_BezierAdaptiveMaxCornerDist; // bezier_adaptive
        float    m_ArcRadius;                   // circular_arc
    };

    // Agent configuration
    struct AgentConfig
    {
        float                       m_MaxSpeed;     // Max movement speed (units/sec)
        float                       m_Radius;       // -> Phase 3/4 Collision radius
        bool                        m_UseAvoidance; // -> Phase 3 Enable collision avoidance
        pathfinder::PathSmoothStyle m_PathSmoothStyle;
        AgentPathSmoothConfig       m_PathSmoothConfig;
    };

    // Agent runtime data - Option 6: Flat Path Array with Indices
    struct Agent
    {
        uint32_t         m_Id;       // Agent ID
        pathfinder::Vec2 m_Position; // Current position
        pathfinder::Vec2 m_Velocity; // Current velocity
        float            m_Rotation; // Current rotation (radians)
        float            m_Speed;    // Current speed
        AgentState       m_State;    // Current state

        // Raw path data (node IDs) - indices into flat array
        uint32_t m_PathStart;          // Start index in m_PathNodeData array
        uint32_t m_PathLength;         // Length of raw path (node IDs)
        uint32_t m_CurrentWaypointIdx; // Current waypoint index (relative to m_PathStart)

        uint32_t m_PathVersion; // Graph version when path computed
        uint32_t m_StartNode;   // Start node
        uint32_t m_GoalNode;    // Destination node

        // Smoothed path data (Vec2 positions) - indices into flat array
        uint32_t m_SmoothedPathStart;          // Start index in m_SmoothedPathData array
        uint32_t m_SmoothedPathLength;         // Length of smoothed path (Vec2 positions)
        uint32_t m_CurrentSmoothedWaypointIdx; // Current index in smoothed path for movement

        // Group data
        uint32_t         m_GroupId;         // Group ID (0 = no group) -> Phase 2
        uint32_t         m_GroupTag;        // Group tag (for filtering) -> Phase 2
        pathfinder::Vec2 m_FormationOffset; // Offset from group center -> Phase 2

        // Configuration
        AgentConfig m_Config;
    };

    // Group data -> Phase 2
    /*   struct AgentGroup
       {
           uint32_t          m_Id;        // Group ID
           uint32_t          m_Tag;       // Group tag (red, blue, etc.)
           pathfinder::Vec2  m_Center;    // Group center position
           dmArray<uint32_t> m_Path;      // Current path for group (node IDs) ->  TODO: Single path is enough for a group
           float             m_Rotation;  // Group rotation
           FormationType     m_Formation; // Formation type
           uint32_t          m_MaxAgents; // Max agents in group
           dmArray<uint32_t> m_AgentIds;  // Agent IDs in group
       };*/
} // namespace navigation

#endif