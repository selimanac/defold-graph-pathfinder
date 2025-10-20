#ifndef NAVIGATION_AGENTS_H
#define NAVIGATION_AGENTS_H

#include "navigation_constants.h"
#include "navigation_types.h"
#include <cstdint>
#include <pathfinder_types.h>

namespace navigation
{
    namespace agents
    {

        // ============================================================================
        // INITIALIZATION
        // ============================================================================

        void init(uint32_t max_agents, uint32_t max_groups, uint32_t max_path_length, uint32_t max_samples_per_segment = 16);

        void shutdown();

        // ============================================================================
        // AGENT MANAGEMENT
        // ============================================================================

        uint32_t         create_agent(pathfinder::Vec2 position, AgentConfig config);

        void             remove_agent(uint32_t agent_id);

        uint32_t         set_agent_target(const uint32_t          agent_id,
                                          const uint32_t          start_node_id,
                                          const uint32_t          goal_node_id,
                                          pathfinder::PathStatus* status,
                                          bool                    use_projected = false); // Not projected, re position the agent at start_node_position

        uint32_t         set_agent_target(const uint32_t          agent_id,
                                          const uint32_t          goal_node_id,
                                          pathfinder::PathStatus* status,
                                          bool                    use_projected = true); // projected

        void             pause_agent(uint32_t agent_id);

        void             resume_agent(uint32_t agent_id, bool replan);

        void             update(float dt);

        pathfinder::Vec2 get_agent_position(uint32_t agent_id);

        pathfinder::Vec2 get_agent_velocity(uint32_t agent_id);

        float            get_agent_rotation(uint32_t agent_id);

        float            get_agent_speed(uint32_t agent_id);

        AgentState       get_agent_state(uint32_t agent_id);

    } // namespace agents
} // namespace navigation

#endif