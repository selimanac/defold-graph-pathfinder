local data  = require("example.scripts.data")
local const = require("example.scripts.const")


-- =================================
-- MODULE
-- =================================
local agents  = {}

local EPSILON = 0.0001



local function get_current_waypoint_position(agent)
	if agent.current_waypoint_id > agent.path_size then
		return agent.position -- No waypoint, stay in place
	end

	local node = agent.path[agent.current_waypoint_id] -- Only for static nodes
	return vmath.vector3(node.x, 0, node.y)
end



function agents.add(start_position, goal_position)
	local path_size = 0
	local path_status = 0
	local path_status_text = ""
	local path = {}

	local goal_node_id = 0

	path_size, path_status, path_status_text, path = pathfinder.navmesh_find_path(start_position.x, start_position.z, goal_position.x, goal_position.z, 128, 0.0)


	pprint(path)

	if path_status ~= pathfinder.PathStatus.SUCCESS then
		return
	end


	path_size, path = pathfinder.smooth_path(data.path_smoothing_id, path)

	local agent_position_v2 = path[1]
	local agent_position = vmath.vector3(agent_position_v2.x, 0, agent_position_v2.y)

	local target_pos_v2 = path[2]
	local target_pos = vmath.vector3(target_pos_v2.x, 0, target_pos_v2.y)

	local to_target = target_pos - agent_position
	local target_rotation = math.atan2(to_target.x, to_target.z)
	local target_quat = vmath.quat_rotation_y(target_rotation)

	local agent = {
		position            = start_position,
		velocity            = vmath.vector3(),
		max_speed           = 5.5,
		rotation_speed      = 10.0,
		speed               = 0,
		rotation            = target_quat,
		rotation_angle      = 0,
		path                = path,
		path_entry_point    = nil,
		path_size           = path_size,
		current_waypoint_id = 1,
		instance            = factory.create("/factories#agent", agent_position, target_quat, nil, vmath.vector3(1.0)),
		model               = "",
		state               = const.AGENT_STATES.ACTIVE,

	}
	agent.model = msg.url(agent.instance)
	agent.model.fragment = "agent"


	table.insert(data.agents, agent)
end

local function check_waypoint_arrival(agent)
	if agent.current_waypoint_id > agent.path_size then
		return false -- No more waypoint
	end

	local waypoint_position = get_current_waypoint_position(agent)
	local waypoint_distance = vmath.length(agent.position - waypoint_position)

	-- Simple arrival threshold - very tight for point-to-point movement
	local arrival_threshold = 0.1

	if waypoint_distance <= arrival_threshold then
		--  Reached waypoint, advance to next
		agent.current_waypoint_id = agent.current_waypoint_id + 1

		if agent.current_waypoint_id > agent.path_size then
			agent.state      = const.AGENT_STATES.ARRIVED
			agent.velocity.x = 0
			agent.velocity.y = 0
			agent.speed      = 0
			return true
		end

		return true -- Advanced to next waypoint
	end

	return false --  Not yet arrived
end

local function draw_path(path_size, path)
	for i = 1, path_size - 1, 1 do
		local from_node = path[i]
		local to_node = path[i + 1]
		msg.post("@render:", "draw_line", { start_point = vmath.vector3(from_node.x, 0, from_node.y), end_point = vmath.vector3(to_node.x, 0, to_node.y), color = const.COLORS.RED })
	end
end




function agents.update(dt)
	for agent_id, agent in ipairs(data.agents) do
		-- Only process active agents
		if agent.state == const.AGENT_STATES.ACTIVE then
			if data.debug then
				draw_path(agent.path_size, agent.path)
			end

			--draw_projected_path(agent.position, agent.path_size, agent.path_entry_point, agent.path)
			-- Check if agent reached current waypoint
			if not check_waypoint_arrival(agent) or agent.state ~= const.AGENT_STATES.ARRIVED then
				-- Get target waypoint position
				local target_pos = get_current_waypoint_position(agent)

				-- Calculate direction to target
				local to_target = target_pos - agent.position
				local distance = vmath.length(to_target)

				if distance >= EPSILON then
					-- Calculate direction unit vector
					local direction = to_target * (1.0 / distance)

					-- Calculate target rotation angle
					local target_rotation = math.atan2(direction.x, direction.z)
					local target_quat = vmath.quat_rotation_y(target_rotation)

					-- Get current rotation quaternion
					local current_quat = agent.rotation --go.get_rotation(agent.instance)

					-- Smooth rotation using slerp
					-- The third parameter (t) controls interpolation speed (0.0 to 1.0)
					-- Lower values = smoother/slower rotation, higher values = faster rotation

					local t = math.min(1.0, agent.rotation_speed * dt)
					agent.rotation = vmath.slerp(t, current_quat, target_quat)

					-- Update agent.rotation for reference (optional, if you need the angle)
					agent.rotation_angle = target_rotation

					-- Calculate movement for this frame and clamp
					local movement_distance = math.min(agent.max_speed * dt, distance)

					-- Update position and rotation
					agent.position = agent.position + (direction * movement_distance)
					agent.position.y = 0.0
					go.set_position(agent.position, agent.instance)
					go.set_rotation(agent.rotation, agent.instance) -- Use smoothed quaternion

					agent.speed = agent.max_speed
				end
			else
				agent.state = const.AGENT_STATES.ARRIVED

				model.play_anim(agent.model, hash("Idle_Loop"), go.PLAYBACK_LOOP_FORWARD)


				go.delete(agent.instance)
				table.remove(data.agents, agent_id)

				print("ARRIVED", agent.state)
			end
		else
			agent.state = const.AGENT_STATES.INACTIVE
			go.delete(agent.instance)
			table.remove(data.agents, agent_id)
		end
	end
end

function agents.get_count()
	return #data.agents
end

return agents
