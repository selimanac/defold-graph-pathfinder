local const = require "example.scripts.const"

-- =================================
-- MODULE
-- =================================
local utils = {}

--==========================================================
-- FUNCTIONS
--==========================================================
function utils.screen_to_plane(cam, sx, sy)
	local p0 = camera.screen_to_world(vmath.vector3(sx, sy, 0), cam)
	local p1 = camera.screen_to_world(vmath.vector3(sx, sy, 1), cam)

	local dir = p1 - p0
	local denom = vmath.dot(const.PLANE_NORMAL, dir)
	if math.abs(denom) < const.EPSILON then
		return nil
	end

	local t = vmath.dot(const.PLANE_POINT - p0, const.PLANE_NORMAL) / denom
	if t < 0 then
		return nil
	end

	return p0 + dir * t
end

-- Get the positions of two nodes connected by an edge
-- Returns two vector3 positions for the start and end of the edge
function utils.get_edge_positions(from_node_id, to_node_id)
	local from_v2 = pathfinder.get_node_position(from_node_id)
	local to_v2 = pathfinder.get_node_position(to_node_id)
	return vmath.vector3(from_v2.x, from_v2.y, 0), vmath.vector3(to_v2.x, to_v2.y, 0)
end

-- Create visual direction indicators for unidirectional edges
-- Bidirectional edges don't need direction indicators
function utils.add_edge_directions(edges)
	for _, edge in ipairs(edges) do
		if edge.bidirectional == false then
			local from, to = utils.get_edge_positions(edge.from_node_id, edge.to_node_id)
			local center = (from + to) * 0.5
			local dir = to - from
			local angle = math.atan2(dir.y, dir.x)

			factory.create(const.FACTORIES.DIRECTION, center, vmath.quat_rotation_z(angle - math.pi * 0.5))
		end
	end
end

function utils.add_nodes(nodes)
	local node_instances = {}
	for _, node_id in ipairs(nodes) do
		local pos_v2 = pathfinder.get_node_position(node_id)
		local node_url = factory.create(const.FACTORIES.NODE, vmath.vector3(pos_v2.x, pos_v2.y, 0))
		table.insert(node_instances, node_id, msg.url(node_url))
		node_url = msg.url(node_url)
		node_url.fragment = "node_id"
		label.set_text(node_url, node_id)
	end

	return node_instances
end

function utils.load_data()
	local edges_json, error = sys.load_resource("/data/edges_1.json")
	if error then
		print("Error loading edges:", error)
		return nil
	end

	local nodes_json, error = sys.load_resource("/data/nodes_1.json")
	if error then
		print("Error loading nodes:", error)
		return nil
	end

	local nodes_data = json.decode(nodes_json)
	local edges_data = json.decode(edges_json)

	return { nodes = nodes_data, edges = edges_data }
end

return utils
