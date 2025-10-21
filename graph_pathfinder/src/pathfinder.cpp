// extension.cpp
// Extension lib defines

#include "dmsdk/dlib/log.h"
#define LIB_NAME "GraphPathfinder"
#define MODULE_NAME "pathfinder"
#include <dmsdk/sdk.h>

// pathfinder
#include "pathfinder_extension.h"
#include "navigation_types.h"
#include "pathfinder_types.h"
#include <pathfinder_path.h>
#include <pathfinder_math.h>
#include "pathfinder_smooth.h"

static inline const char* path_status_to_string(enum pathfinder::PathStatus status)
{
    switch (status)
    {
        case pathfinder::SUCCESS:
            return "Success";
        case pathfinder::ERROR_NO_PATH:
            return "No valid path found between start and goal nodes";
        case pathfinder::ERROR_START_NODE_INVALID:
            return "Invalid or inactive start node ID";
        case pathfinder::ERROR_GOAL_NODE_INVALID:
            return "Invalid or inactive goal node ID";
        case pathfinder::ERROR_NODE_FULL:
            return "Node capacity reached — cannot add more nodes";
        case pathfinder::ERROR_EDGE_FULL:
            return "Edge capacity reached — cannot add more edges";
        case pathfinder::ERROR_HEAP_FULL:
            return "Heap pool exhausted during pathfinding (increase pool size)";
        case pathfinder::ERROR_PATH_TOO_LONG:
            return "Path exceeds maximum allowed length";
        case pathfinder::ERROR_GRAPH_CHANGED:
            return "Graph modified during pathfinding — retrying";
        case pathfinder::ERROR_GRAPH_CHANGED_TOO_OFTEN:
            return "Graph changed too often during pathfinding (>3 retries)";
        case pathfinder::ERROR_NO_PROJECTION:
            return "Cannot project point onto graph (no edges exist)";
        case pathfinder::ERROR_VIRTUAL_NODE_FAILED:
            return "Failed to create or connect virtual node";
        default:
            return "Unknown pathfinding error";
    }
}

// Helper function to create a Lua table with path waypoints (node positions with optional IDs)
static void push_path_node_table(lua_State* L, const pathfinder::Vec2& pos, uint32_t node_id = 0, bool include_id = false)
{
    lua_createtable(L, 0, include_id ? 3 : 2);

    lua_pushstring(L, "x");
    lua_pushnumber(L, pos.x);
    lua_settable(L, -3);

    lua_pushstring(L, "y");
    lua_pushnumber(L, pos.y);
    lua_settable(L, -3);

    if (include_id)
    {
        lua_pushstring(L, "id");
        lua_pushinteger(L, node_id);
        lua_settable(L, -3);
    }
}

// Helper function to create a Lua table array from smoothed path positions
static void push_smoothed_path_table(lua_State* L, const dmArray<pathfinder::Vec2>& smoothed_path)
{
    lua_createtable(L, smoothed_path.Size(), 0);
    int newTable = lua_gettop(L);
    for (int i = 0; i < smoothed_path.Size(); ++i)
    {
        push_path_node_table(L, smoothed_path[i]);
        lua_rawseti(L, newTable, i + 1);
    }
}

static int pathfinder_init(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 0);

    uint32_t max_nodes = luaL_checkint(L, 1);
    uint32_t max_gameobject_nodes = (uint32_t)luaL_optinteger(L, 2, 0);
    uint32_t max_edge_per_node = luaL_checkint(L, 3);
    uint32_t pool_block_size = luaL_checkint(L, 4);
    uint32_t max_cache_path_length = luaL_checkint(L, 5);

    pathfinder::path::init(max_nodes, max_edge_per_node, pool_block_size, max_cache_path_length);

    if (max_gameobject_nodes > 0)
    {
        pathfinder::extension::set_gameobject_capacity(max_gameobject_nodes);
    }
    return 0;
}

static int pathfinder_add_nodes(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 1);
    // IN  <-
    luaL_checktype(L, 1, LUA_TTABLE);

    int node_count = (int)lua_objlen(L, 1);
    dmLogInfo("Adding %d nodes", node_count);

    pathfinder::PathStatus status;

    dmArray<uint32_t>      node_ids;
    node_ids.SetCapacity(node_count);

    for (int i = 1; i <= node_count; ++i)
    {
        lua_rawgeti(L, 1, i); // push node_positions[i] onto stack

        if (lua_istable(L, -1))
        {
            lua_getfield(L, -1, "x");
            float x = luaL_checknumber(L, -1);
            lua_pop(L, 1);

            lua_getfield(L, -1, "y");
            float y = luaL_checknumber(L, -1);
            lua_pop(L, 1);

            pathfinder::Vec2 pos = pathfinder::Vec2(x, y);
            uint32_t         node_id = pathfinder::path::add_node(pos, &status);

            if (status != pathfinder::SUCCESS)
            {
                dmLogError("Node %d: x=%.1f, y=%.1f Failed. %s  (status: %d)", i, x, y, path_status_to_string(status), status);
            }
            else
            {
                node_ids.Push(node_id);
            }
        }

        lua_pop(L, 1); // pop inner table
    }

    // OUT ->
    lua_createtable(L, node_ids.Size(), 0);
    for (int i = 0; i < node_ids.Size(); ++i)
    {
        lua_pushinteger(L, node_ids[i]);
        lua_rawseti(L, -2, i + 1);
    }

    return 1;
}

static int pathfinder_add_gameobject_node(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 1);

    // IN <<-
    dmGameObject::HInstance gameobject_instance = dmScript::CheckGOInstance(L, 1);
    bool                    use_world_position = lua_toboolean(L, 2);

    dmVMath::Point3         gameobject_position = dmGameObject::GetPosition(gameobject_instance);
    pathfinder::Vec2        pos = pathfinder::Vec2(gameobject_position.getX(), gameobject_position.getY());

    pathfinder::PathStatus  status;
    uint32_t                node_id = pathfinder::path::add_node(pos, &status);
    pathfinder::extension::add_gameobject_node(node_id, gameobject_instance, gameobject_position, use_world_position);

    // OUT ->>
    lua_pushinteger(L, node_id);

    if (status != pathfinder::SUCCESS)
    {
        dmLogError("Failed. %s  (status: %d)", path_status_to_string(status), status);
    }

    return 1;
}

static int pathfinder_add_gameobject_nodes(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 1);
    luaL_checktype(L, 1, LUA_TTABLE);

    int gameobject_count = (int)lua_objlen(L, 1);
    dmLogInfo("Adding %d gameobject nodes", gameobject_count);

    pathfinder::PathStatus status;
    dmArray<uint32_t>      node_ids;
    node_ids.SetCapacity(gameobject_count);

    for (int i = 1; i <= gameobject_count; ++i)
    {
        lua_rawgeti(L, 1, i); // push nodes[i]
        if (!lua_istable(L, -1))
        {
            lua_pop(L, 1);
            continue;
        }
        // Get nodes[i][1] = msg.url(...)
        lua_rawgeti(L, -1, 1);
        dmGameObject::HInstance go_instance = dmScript::CheckGOInstance(L, -1);
        lua_pop(L, 1); // pop url

        // Get nodes[i][2] = optional boolean
        lua_rawgeti(L, -1, 2);
        bool use_world_position = lua_toboolean(L, -1); // false if nil
        lua_pop(L, 1);                                  // pop bool

        // Now process gameobject
        dmVMath::Point3  gameobject_position = dmGameObject::GetPosition(go_instance);
        pathfinder::Vec2 pos(gameobject_position.getX(), gameobject_position.getY());

        uint32_t         node_id = pathfinder::path::add_node(pos, &status);
        pathfinder::extension::add_gameobject_node(node_id, go_instance, gameobject_position, use_world_position);

        if (status != pathfinder::SUCCESS)
        {
            dmLogError("Node %d: x=%.1f, y=%.1f Failed (%s)", i, pos.x, pos.y, path_status_to_string(status));
        }
        else
        {
            node_ids.Push(node_id);
        }

        lua_pop(L, 1); // pop nodes[i]
    }

    // Return array of node IDs
    lua_createtable(L, node_ids.Size(), 0);
    for (int i = 0; i < node_ids.Size(); ++i)
    {
        lua_pushinteger(L, node_ids[i]);
        lua_rawseti(L, -2, i + 1);
    }

    return 1;
}

static int pathfinder_convert_gameobject_node(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 0);

    // IN <<-
    uint32_t                node_id = luaL_checkinteger(L, 1);
    dmGameObject::HInstance gameobject_instance = dmScript::CheckGOInstance(L, 2);
    bool                    use_world_position = lua_toboolean(L, 3);

    dmVMath::Point3         gameobject_position = dmGameObject::GetPosition(gameobject_instance);
    pathfinder::Vec2        pos = pathfinder::Vec2(gameobject_position.getX(), gameobject_position.getY());

    pathfinder::extension::add_gameobject_node(node_id, gameobject_instance, gameobject_position, use_world_position);

    return 0;
}

static int pathfinder_add_node(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 1);

    float                  x = luaL_checknumber(L, 1);
    float                  y = luaL_checknumber(L, 2);
    pathfinder::Vec2       pos = pathfinder::Vec2(x, y);

    pathfinder::PathStatus status;

    uint32_t               node_id = pathfinder::path::add_node(pos, &status);

    lua_pushinteger(L, node_id);

    if (status != pathfinder::SUCCESS)
    {
        dmLogError("Failed. %s  (status: %d)", path_status_to_string(status), status);
    }

    return 1;
}

static int pathfinder_add_edges(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 0);

    // IN  <-
    luaL_checktype(L, 1, LUA_TTABLE);

    int edge_count = (int)lua_objlen(L, 1);
    dmLogInfo("Adding %d edges", edge_count);

    pathfinder::PathStatus status;

    for (int i = 1; i <= edge_count; ++i)
    {
        lua_rawgeti(L, 1, i); // push edges[i] onto stack

        if (lua_istable(L, -1))
        {
            lua_getfield(L, -1, "from_node_id");
            uint32_t from_node_id = luaL_checkint(L, -1);
            lua_pop(L, 1);

            lua_getfield(L, -1, "to_node_id");
            uint32_t to_node_id = luaL_checkint(L, -1);
            lua_pop(L, 1);

            lua_getfield(L, -1, "bidirectional");
            bool bidirectional = lua_toboolean(L, -1);
            lua_pop(L, 1);

            // Optional cost
            lua_getfield(L, -1, "cost");
            float cost;
            if (!lua_isnoneornil(L, -1))
            {
                cost = (float)luaL_checknumber(L, -1);
            }
            else
            {
                cost = pathfinder::math::distance(
                pathfinder::path::get_node_position(from_node_id),
                pathfinder::path::get_node_position(to_node_id));
            }
            lua_pop(L, 1);
            // -----------------------------

            pathfinder::PathStatus status;
            pathfinder::path::add_edge(from_node_id, to_node_id, cost, bidirectional, &status);

            if (status != pathfinder::SUCCESS)
            {
                dmLogError("Edge %d: from_node_id=%u, to_node_id=%u Failed. %s (status: %d)",
                           i,
                           from_node_id,
                           to_node_id,
                           path_status_to_string(status),
                           status);
            }
        }

        lua_pop(L, 1); // pop inner table
    }

    return 0;
}

static int pathfinder_add_edge(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 0);

    uint32_t from_node_id = luaL_checkint(L, 1);
    uint32_t to_node_id = luaL_checkint(L, 2);
    bool     bidirectional = lua_toboolean(L, 3);

    float    cost;

    // If 4th argument is given and not nil, use it
    if (!lua_isnoneornil(L, 4))
    {
        cost = (float)luaL_checknumber(L, 4);
    }
    else
    {
        // Default cost = distance between nodes
        cost = pathfinder::math::distance(
        pathfinder::path::get_node_position(from_node_id),
        pathfinder::path::get_node_position(to_node_id));
    }

    pathfinder::PathStatus status;
    pathfinder::path::add_edge(from_node_id, to_node_id, cost, bidirectional, &status);

    if (status != pathfinder::SUCCESS)
    {
        dmLogError("Failed. %s (status: %d)", path_status_to_string(status), status);
    }

    return 0;
}

static int pathfinder_find_path(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 4);

    // IN <-
    uint32_t start_node_id = luaL_checkint(L, 1);
    uint32_t goal_node_id = luaL_checkint(L, 2);
    uint32_t max_path = luaL_checkint(L, 3);
    uint32_t smooth_id = (uint32_t)luaL_optinteger(L, 4, 0);

    // OUT ->
    dmArray<uint32_t>      path;
    pathfinder::PathStatus status;
    uint32_t               path_length = pathfinder::path::find_path(start_node_id, goal_node_id, &path, max_path, &status);

    // Smoothing
    if (smooth_id > 0)
    {
        dmArray<pathfinder::Vec2> smoothed_path;
        uint32_t                  samples_per_segment = pathfinder::extension::get_smooth_sample_segment(smooth_id);
        uint32_t                  capacity = pathfinder::smooth::calculate_smoothed_path_capacity(path, samples_per_segment);
        smoothed_path.SetCapacity(capacity);

        pathfinder::extension::smooth_path(smooth_id, path, smoothed_path);

        lua_pushinteger(L, smoothed_path.Size());
        lua_pushinteger(L, status);
        lua_pushstring(L, path_status_to_string(status));

        // Result table
        push_smoothed_path_table(L, smoothed_path);
    }
    else
    {
        lua_pushinteger(L, path_length);
        lua_pushinteger(L, status);
        lua_pushstring(L, path_status_to_string(status));

        // Result table
        lua_createtable(L, path_length, 0);
        int newTable = lua_gettop(L);
        for (int i = 0; i < path_length; ++i)
        {
            pathfinder::Vec2 node_position = pathfinder::path::get_node_position(path[i]);
            push_path_node_table(L, node_position, path[i], true);
            lua_rawseti(L, newTable, i + 1);
        }
    }

    return 4;
}

static int pathfinder_find_projected_path(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 5);

    // IN <-
    float            x = luaL_checknumber(L, 1);
    float            y = luaL_checknumber(L, 2);
    pathfinder::Vec2 pos = pathfinder::Vec2(x, y);

    uint32_t         goal_node_id = luaL_checkint(L, 3);
    uint32_t         max_path = luaL_checkint(L, 4);
    uint32_t         smooth_id = (uint32_t)luaL_optinteger(L, 5, 0);

    // OUT ->
    dmArray<uint32_t>      path;
    pathfinder::PathStatus status;
    pathfinder::Vec2       entry_point;
    uint32_t               path_length = pathfinder::path::find_path_projected(pos, goal_node_id, &path, max_path, &entry_point, &status);

    // Smoothing
    if (smooth_id > 0)
    {
        dmArray<pathfinder::Vec2> waypoints;
        waypoints.SetCapacity(path_length + 2);
        waypoints.Push(pos);         // Start position
        waypoints.Push(entry_point); // Entry point on graph

        // Add all path nodes
        for (uint32_t i = 0; i < path_length; i++)
        {
            waypoints.Push(pathfinder::path::get_node_position(path[i]));
        }

        dmArray<pathfinder::Vec2> smoothed_path;
        uint32_t                  samples_per_segment = pathfinder::extension::get_smooth_sample_segment(smooth_id);
        uint32_t                  capacity = pathfinder::smooth::calculate_smoothed_path_capacity(path, samples_per_segment);
        smoothed_path.SetCapacity(capacity);

        pathfinder::extension::smooth_path_waypoint(smooth_id, waypoints, smoothed_path);

        lua_pushinteger(L, smoothed_path.Size());
        lua_pushinteger(L, status);
        lua_pushstring(L, path_status_to_string(status));
        dmScript::PushVector3(L, dmVMath::Vector3(entry_point.x, entry_point.y, 0));

        // Result table
        push_smoothed_path_table(L, smoothed_path);
    }
    else
    {
        lua_pushinteger(L, path_length);
        lua_pushinteger(L, status);
        lua_pushstring(L, path_status_to_string(status));
        dmScript::PushVector3(L, dmVMath::Vector3(entry_point.x, entry_point.y, 0));

        // Result table
        lua_createtable(L, path_length, 0);
        int newTable = lua_gettop(L);
        for (int i = 0; i < path_length; i++)
        {
            pathfinder::Vec2 node_position = pathfinder::path::get_node_position(path[i]);
            push_path_node_table(L, node_position, path[i], true);
            lua_rawseti(L, newTable, i + 1);
        }
    }
    return 5;
}

static int pathfinder_shutdown(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 0);

    pathfinder::path::shutdown();

    return 0;
}

static int pathfinder_remove_node(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 0);
    uint32_t node_id = luaL_checkint(L, 1);
    pathfinder::path::remove_node(node_id);
    return 0;
}

static int pathfinder_remove_gameobject_node(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 0);
    uint32_t node_id = luaL_checkint(L, 1);
    pathfinder::extension::remove_gameobject_node(node_id);
    pathfinder::path::remove_node(node_id);
    return 0;
}

static int pathfinder_get_node_position(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 1);
    uint32_t         node_id = luaL_checkint(L, 1);
    pathfinder::Vec2 pos = pathfinder::path::get_node_position(node_id);

    // Create a table with 0 array slots, 2 hash entries
    lua_createtable(L, 0, 2);

    lua_pushnumber(L, pos.x);
    lua_setfield(L, -2, "x");

    lua_pushnumber(L, pos.y);
    lua_setfield(L, -2, "y");

    return 1; // the table
}

static int pathfinder_remove_edge(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 0);

    uint32_t from = luaL_checkinteger(L, 1);
    uint32_t to = luaL_checkinteger(L, 2);

    pathfinder::path::remove_edge(from, to);
    if (lua_toboolean(L, 3))
        pathfinder::path::remove_edge(to, from);

    return 0;
}

static int pathfinder_move_node(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 0);

    uint32_t         node_id = luaL_checkinteger(L, 1);
    float            x = luaL_checknumber(L, 2);
    float            y = luaL_checknumber(L, 3);
    pathfinder::Vec2 pos = pathfinder::Vec2(x, y);

    pathfinder::path::move_node(node_id, pos);

    return 0;
}

static int pathfinder_add_path_smoothing(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 1);

    // IN <-
    luaL_checktype(L, 1, LUA_TTABLE);

    navigation::AgentPathSmoothConfig path_smooth_config;

    // Get "style"
    lua_getfield(L, 1, "style");
    uint32_t smooth_style = luaL_checkinteger(L, -1);
    lua_pop(L, 1);

    // Get "bezier_sample_segment"
    lua_getfield(L, 1, "bezier_sample_segment");
    path_smooth_config.m_SampleSegment = luaL_optinteger(L, -1, 0);
    lua_pop(L, 1);

    // Get "bezier_control_point_offset"
    lua_getfield(L, 1, "bezier_control_point_offset");
    path_smooth_config.m_ControlPointOffset = (float)luaL_optnumber(L, -1, 0.0);
    lua_pop(L, 1);

    // Get "bezier_curve_radius"
    lua_getfield(L, 1, "bezier_curve_radius");
    path_smooth_config.m_CurveRadius = (float)luaL_optnumber(L, -1, 0.0);
    lua_pop(L, 1);

    // Get "bezier_adaptive_tightness"
    lua_getfield(L, 1, "bezier_adaptive_tightness");
    path_smooth_config.m_BezierAdaptiveTightness = (float)luaL_optnumber(L, -1, 0.0);
    lua_pop(L, 1);

    // Get "bezier_adaptive_roundness"
    lua_getfield(L, 1, "bezier_adaptive_roundness");
    path_smooth_config.m_BezierAdaptiveRoundness = (float)luaL_optnumber(L, -1, 0.0);
    lua_pop(L, 1);

    // Get "bezier_adaptive_max_corner_distance"
    lua_getfield(L, 1, "bezier_adaptive_max_corner_distance");
    path_smooth_config.m_BezierAdaptiveMaxCornerDist = (float)luaL_optnumber(L, -1, 0.0);
    lua_pop(L, 1);

    // Get "bezier_arc_radius"
    lua_getfield(L, 1, "bezier_arc_radius");
    path_smooth_config.m_ArcRadius = (float)luaL_optnumber(L, -1, 0.0);
    lua_pop(L, 1);

    // OUT ->
    uint32_t smooth_id = pathfinder::extension::add_smooth_config(smooth_style, path_smooth_config);
    lua_pushinteger(L, smooth_id);
    return 1;
}

static int pathfinder_smooth_path(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 2);

    // IN <<-
    uint32_t smooth_id = luaL_checkinteger(L, 1);
    luaL_checktype(L, 2, LUA_TTABLE);

    int                       path_count = (int)lua_objlen(L, 2);
    dmArray<pathfinder::Vec2> waypoints;
    waypoints.SetCapacity(path_count);

    for (int i = 1; i <= path_count; ++i)
    {
        lua_rawgeti(L, 2, i);

        if (lua_istable(L, -1))
        {
            lua_getfield(L, -1, "x");
            float x = luaL_checknumber(L, -1);
            lua_pop(L, 1);

            lua_getfield(L, -1, "y");
            float y = luaL_checknumber(L, -1);
            lua_pop(L, 1);

            pathfinder::Vec2 pos = pathfinder::Vec2(x, y);
            waypoints.Push(pos);
        }

        lua_pop(L, 1); // pop inner table
    }

    dmArray<pathfinder::Vec2> smoothed_path;
    uint32_t                  samples_per_segment = pathfinder::extension::get_smooth_sample_segment(smooth_id);

    smoothed_path.SetCapacity(path_count * samples_per_segment);

    pathfinder::extension::smooth_path_waypoint(smooth_id, waypoints, smoothed_path);

    // OUT ->>
    lua_pushinteger(L, smoothed_path.Size());

    // Result table
    push_smoothed_path_table(L, smoothed_path);

    return 2;
}

static int pathfinder_set_gameobject_update(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 0);
    bool state = lua_toboolean(L, 1);
    pathfinder::extension::set_update_state(state);
    return 0;
}

static int pathfinder_pause_gameobject_node(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 0);
    uint32_t node_id = luaL_checkinteger(L, 1);
    pathfinder::extension::pause_gameobject_node(node_id);
    return 0;
}

static int pathfinder_resume_gameobject_node(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 0);
    uint32_t node_id = luaL_checkinteger(L, 1);
    pathfinder::extension::resume_gameobject_node(node_id);
    return 0;
}

static int pathfinder_set_update_frequency(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 0);
    uint8_t frequency = luaL_checknumber(L, 1);
    pathfinder::extension::set_update_frequency(frequency);
    return 0;
}

static int pathfinder_cache_stats(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 1);

    // ============================================================================
    // CACHE STATISTICS VARIABLES
    // ============================================================================
    uint32_t path_cache_entries;  // Path cache: current entries
    uint32_t path_cache_capacity; // Path cache: max capacity
    uint32_t path_cache_hit_rate; // Path cache: hit rate percentage

    uint32_t dist_cache_size;     // Distance cache: current size
    uint32_t dist_cache_hits;     // Distance cache: hit count
    uint32_t dist_cache_misses;   // Distance cache: miss count
    uint32_t dist_cache_hit_rate; // Distance cache: hit rate percentage

    pathfinder::extension::get_cache_stats(path_cache_entries, path_cache_capacity, path_cache_hit_rate, dist_cache_size, dist_cache_hits, dist_cache_misses, dist_cache_hit_rate);
    // ============================================================================
    // CREATE RESULT TABLE
    // ============================================================================
    lua_createtable(L, 0, 2); // main table (2 hash fields: path_cache, distance_cache)

    //  path_cache
    lua_createtable(L, 0, 3);
    lua_pushinteger(L, path_cache_entries);
    lua_setfield(L, -2, "current_entries");
    lua_pushinteger(L, path_cache_capacity);
    lua_setfield(L, -2, "max_capacity");
    lua_pushinteger(L, path_cache_hit_rate);
    lua_setfield(L, -2, "hit_rate");

    // add to main table
    lua_setfield(L, -2, "path_cache");

    //  distance_cache
    lua_createtable(L, 0, 4);
    lua_pushinteger(L, dist_cache_size);
    lua_setfield(L, -2, "current_size");
    lua_pushinteger(L, dist_cache_hits);
    lua_setfield(L, -2, "hit_count");
    lua_pushinteger(L, dist_cache_misses);
    lua_setfield(L, -2, "miss_count");
    lua_pushinteger(L, dist_cache_hit_rate);
    lua_setfield(L, -2, "hit_rate");

    // add to main table
    lua_setfield(L, -2, "distance_cache");

    return 1; // one table on stack
}

// Functions exposed to Lua
static const luaL_reg Module_methods[] = {

    // OPs
    { "init", pathfinder_init },
    { "shutdown", pathfinder_shutdown },
    { "get_cache_stats", pathfinder_cache_stats },

    // Nodes
    { "add_node", pathfinder_add_node },
    { "add_nodes", pathfinder_add_nodes },
    { "remove_node", pathfinder_remove_node },
    { "move_node", pathfinder_move_node },
    { "get_node_position", pathfinder_get_node_position },

    // Edges
    { "add_edge", pathfinder_add_edge },
    { "add_edges", pathfinder_add_edges },
    { "remove_edge", pathfinder_remove_edge },

    // Path
    { "find_path", pathfinder_find_path },
    { "find_projected_path", pathfinder_find_projected_path },

    // Smooth
    { "smooth_path", pathfinder_smooth_path },
    { "add_path_smoothing", pathfinder_add_path_smoothing },

    // Gameobjects
    { "add_gameobject_node", pathfinder_add_gameobject_node },
    { "add_gameobject_nodes", pathfinder_add_gameobject_nodes },
    { "convert_gameobject_node", pathfinder_convert_gameobject_node },
    { "remove_gameobject_node", pathfinder_remove_gameobject_node },
    { "pause_gameobject_node", pathfinder_pause_gameobject_node },
    { "resume_gameobject_node", pathfinder_resume_gameobject_node },

    // Update
    { "gameobject_update", pathfinder_set_gameobject_update },
    { "set_update_frequency", pathfinder_set_update_frequency },

    { 0, 0 }
};

static void LuaInit(lua_State* L)
{
    int top = lua_gettop(L);

    // Register module functions
    luaL_register(L, MODULE_NAME, Module_methods);

    // ----------------------------
    // PathStatus enum
    // ----------------------------
    lua_newtable(L); // PathStatus table
    int pathStatusTable = lua_gettop(L);
#define SET_CONSTANT(table, name) \
    lua_pushnumber(L, (lua_Number)pathfinder::name); \
    lua_setfield(L, table, #name)

    SET_CONSTANT(pathStatusTable, SUCCESS);
    SET_CONSTANT(pathStatusTable, ERROR_NO_PATH);
    SET_CONSTANT(pathStatusTable, ERROR_START_NODE_INVALID);
    SET_CONSTANT(pathStatusTable, ERROR_GOAL_NODE_INVALID);
    SET_CONSTANT(pathStatusTable, ERROR_NODE_FULL);
    SET_CONSTANT(pathStatusTable, ERROR_EDGE_FULL);
    SET_CONSTANT(pathStatusTable, ERROR_HEAP_FULL);
    SET_CONSTANT(pathStatusTable, ERROR_PATH_TOO_LONG);
    SET_CONSTANT(pathStatusTable, ERROR_GRAPH_CHANGED);
    SET_CONSTANT(pathStatusTable, ERROR_GRAPH_CHANGED_TOO_OFTEN);
    SET_CONSTANT(pathStatusTable, ERROR_NO_PROJECTION);
    SET_CONSTANT(pathStatusTable, ERROR_VIRTUAL_NODE_FAILED);

#undef SET_CONSTANT
    lua_setfield(L, -2, "PathStatus"); // attach as module.PathStatus

    // ----------------------------
    // PathSmoothStyle enum
    // ----------------------------
    lua_newtable(L); // PathSmoothStyle table
    int pathStyleTable = lua_gettop(L);
#define SET_CONSTANT(table, name) \
    lua_pushnumber(L, (lua_Number)pathfinder::name); \
    lua_setfield(L, table, #name)

    SET_CONSTANT(pathStyleTable, NONE);
    SET_CONSTANT(pathStyleTable, CATMULL_ROM);
    SET_CONSTANT(pathStyleTable, BEZIER_CUBIC);
    SET_CONSTANT(pathStyleTable, BEZIER_QUADRATIC);
    SET_CONSTANT(pathStyleTable, BEZIER_ADAPTIVE);
    SET_CONSTANT(pathStyleTable, CIRCULAR_ARC);

#undef SET_CONSTANT
    lua_setfield(L, -2, "PathSmoothStyle"); // attach as module.PathSmoothStyle

    lua_pop(L, 1); // pop module table
    assert(top == lua_gettop(L));
}

static dmExtension::Result AppInitializeGraphPathfinder(dmExtension::AppParams* params)
{
    dmLogInfo("AppInitializeGraphPathfinder");
    uint8_t update_frequency = dmConfigFile::GetInt(params->m_ConfigFile, "display.update_frequency", 0);

    pathfinder::extension::init();
    pathfinder::extension::set_update_frequency(update_frequency);

    return dmExtension::RESULT_OK;
}

static dmExtension::Result InitializeGraphPathfinder(dmExtension::Params* params)
{
    // Init Lua
    LuaInit(params->m_L);
    dmLogInfo("Registered %s Extension", MODULE_NAME);
    return dmExtension::RESULT_OK;
}

static dmExtension::Result AppFinalizeGraphPathfinder(dmExtension::AppParams* params)
{
    dmLogInfo("AppFinalizeGraphPathfinder");
    pathfinder::extension::shutdown();
    pathfinder::path::shutdown();
    return dmExtension::RESULT_OK;
}

static dmExtension::Result OnUpdateGraphPathfinder(dmExtension::Params* params)
{
    pathfinder::extension::update();
    return dmExtension::RESULT_OK;
}

// Defold SDK uses a macro for setting up extension entry points:
//
// DM_DECLARE_EXTENSION(symbol, name, app_init, app_final, init, update, on_event, final)

// GraphPathfinder is the C++ symbol that holds all relevant extension data.
// It must match the name field in the `ext.manifest`
DM_DECLARE_EXTENSION(GraphPathfinder, LIB_NAME, AppInitializeGraphPathfinder, AppFinalizeGraphPathfinder, InitializeGraphPathfinder, OnUpdateGraphPathfinder, 0, 0)
