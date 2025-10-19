// extension.cpp
// Extension lib defines
#include "dmarray.h"
#include "dmsdk/dlib/log.h"
#include "dmsdk/script/script.h"
#include "pathfinder_types.h"
#include <cstdio>
#define LIB_NAME "GraphPathfinder"
#define MODULE_NAME "pathfinder"

// include the Defold SDK
#include <dmsdk/sdk.h>
#include <pathfinder_path.h>
#include <pathfinder_math.h>

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

static int pathfinder_init(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 0);

    uint32_t max_nodes = luaL_checkint(L, 1);
    uint32_t max_edge_per_node = luaL_checkint(L, 2);
    uint32_t pool_block_size = luaL_checkint(L, 3);
    uint32_t max_cache_path_length = luaL_checkint(L, 4);
    pathfinder::path::init(max_nodes, max_edge_per_node, pool_block_size, max_cache_path_length);

    return 0;
}

/*static int pathfinder_add_nodes(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 0);

    luaL_checktype(L, 1, LUA_TTABLE);

    int count = (int)lua_objlen(L, 1);
    dmLogInfo("Adding %d nodes", count);

    lua_pushnil(L);
    // for pairs -> order not guaranteed
    while (lua_next(L, 1) != 0)
    {
        if (lua_istable(L, -1))
        {
            lua_getfield(L, -1, "x");
            lua_getfield(L, -2, "y");

            float x = luaL_checknumber(L, -2);
            float y = luaL_checknumber(L, -1);

            dmLogInfo("x %.1f - y %.1f", x, y);

            lua_pop(L, 2); // pop x, y
        }

        lua_pop(L, 1); // pop value (inner table), keep key for next iteration
    }

    return 0;
}*/

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
        lua_rawgeti(L, 1, i); // push node_positions[i] onto stack

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

            float cost = pathfinder::math::distance(pathfinder::path::get_node_position(from_node_id), pathfinder::path::get_node_position(to_node_id));

            pathfinder::path::add_edge(from_node_id, to_node_id, cost, bidirectional, &status);

            if (status != pathfinder::SUCCESS)
            {
                dmLogError("Edge %d: from_node_id=%u, to_node_id=%u Failed. %s  (status: %d)", i, from_node_id, to_node_id, path_status_to_string(status), status);
            }
        }

        lua_pop(L, 1); // pop inner table
    }

    return 0;
}

static int pathfinder_add_edge(lua_State* L)
{
    DM_LUA_STACK_CHECK(L, 0);

    uint32_t               from_node_id = luaL_checkint(L, 1);
    uint32_t               to_node_id = luaL_checkint(L, 2);
    bool                   bidirectional = lua_toboolean(L, 3);

    pathfinder::PathStatus status;
    float                  cost = pathfinder::math::distance(pathfinder::path::get_node_position(from_node_id), pathfinder::path::get_node_position(to_node_id));

    pathfinder::path::add_edge(from_node_id, to_node_id, cost, bidirectional, &status);

    if (status != pathfinder::SUCCESS)
    {
        dmLogError("Failed. %s  (status: %d)", path_status_to_string(status), status);
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

    // OUT ->
    dmArray<uint32_t>      path;
    pathfinder::PathStatus status;
    uint32_t               path_length = pathfinder::path::find_path(start_node_id, goal_node_id, &path, max_path, &status);

    lua_pushinteger(L, path_length);
    lua_pushinteger(L, status);
    lua_pushstring(L, path_status_to_string(status));

    lua_createtable(L, path_length, 0);
    int newTable = lua_gettop(L);

    for (int ii = 0; ii < path_length; ++ii)
    {
        pathfinder::Vec2 node_position = pathfinder::path::get_node_position(path[ii]);

        lua_createtable(L, 0, 3);

        lua_pushstring(L, "x");
        lua_pushinteger(L, node_position.x);
        lua_settable(L, -3);

        lua_pushstring(L, "y");
        lua_pushinteger(L, node_position.y);
        lua_settable(L, -3);

        lua_pushstring(L, "id");
        lua_pushinteger(L, path[ii]);
        lua_settable(L, -3);

        lua_rawseti(L, newTable, ii + 1);
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

    // OUT ->
    dmArray<uint32_t>      path;
    pathfinder::PathStatus status;
    pathfinder::Vec2       entry_point;
    uint32_t               path_length = pathfinder::path::find_path_projected(pos, goal_node_id, &path, max_path, &entry_point, &status);

    lua_pushinteger(L, path_length);
    lua_pushinteger(L, status);
    lua_pushstring(L, path_status_to_string(status));

    dmScript::PushVector3(L, dmVMath::Vector3(entry_point.x, entry_point.y, 0));

    lua_createtable(L, path_length, 0);
    int newTable = lua_gettop(L);
    for (int ii = 0; ii < path_length; ii++)
    {
        pathfinder::Vec2 node_position = pathfinder::path::get_node_position(path[ii]);

        lua_createtable(L, 2, 0);
        lua_pushstring(L, "x");
        lua_pushinteger(L, node_position.x);
        lua_settable(L, -3);
        lua_pushstring(L, "y");
        lua_pushinteger(L, node_position.y);
        lua_settable(L, -3);
        lua_pushstring(L, "id");
        lua_pushinteger(L, path[ii]);
        lua_settable(L, -3);

        lua_rawseti(L, newTable, ii + 1);
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

// Functions exposed to Lua
static const luaL_reg Module_methods[] = {
    { "init", pathfinder_init },
    { "add_node", pathfinder_add_node },
    { "add_edge", pathfinder_add_edge },
    { "remove_node", pathfinder_remove_node },
    { "remove_edge", pathfinder_remove_edge },
    { "move_node", pathfinder_move_node },
    { "add_nodes", pathfinder_add_nodes },
    { "add_edges", pathfinder_add_edges },
    { "find_path", pathfinder_find_path },
    { "find_projected_path", pathfinder_find_projected_path },
    { "shutdown", pathfinder_shutdown },
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
    return dmExtension::RESULT_OK;
}

static dmExtension::Result FinalizeGraphPathfinder(dmExtension::Params* params)
{
    dmLogInfo("FinalizeGraphPathfinder");
    return dmExtension::RESULT_OK;
}

/*static dmExtension::Result OnUpdateGraphPathfinder(dmExtension::Params* params)
{
    dmLogInfo("OnUpdateGraphPathfinder");
    return dmExtension::RESULT_OK;
}*/

// Defold SDK uses a macro for setting up extension entry points:
//
// DM_DECLARE_EXTENSION(symbol, name, app_init, app_final, init, update, on_event, final)

// GraphPathfinder is the C++ symbol that holds all relevant extension data.
// It must match the name field in the `ext.manifest`
DM_DECLARE_EXTENSION(GraphPathfinder, LIB_NAME, AppInitializeGraphPathfinder, AppFinalizeGraphPathfinder, InitializeGraphPathfinder, 0, 0, FinalizeGraphPathfinder)
