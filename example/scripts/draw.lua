local const = require "example.scripts.const"
local utils = require "example.scripts.utils"

-- =================================
-- MODULE
-- =================================
local draw  = {}

--==========================================================
-- FUNCTIONS
--==========================================================
function draw.node_to_node(path_size, path, y_axis)
	for i = 1, path_size - 1 do
		local a, b = path[i], path[i + 1]
		msg.post("@render:", "draw_line", {
			start_point = vmath.vector3(a.x, y_axis and a.y or 0, y_axis and 0 or a.y),
			end_point   = vmath.vector3(b.x, y_axis and b.y or 0, y_axis and 0 or b.y),
			color       = const.COLORS.RED
		})
	end
end

function draw.grid(grid)
	for _, direction in pairs(grid) do
		for _, cell in ipairs(direction) do
			msg.post("@render:", "draw_line", { start_point = cell.start_position, end_point = cell.end_position, color = const.COLORS.RED })
		end
	end
end

-- Draw all edges in the graph as green lines
function draw.edges(edges)
	for _, edge in ipairs(edges) do
		local from, to = utils.get_edge_positions(edge.from_node_id, edge.to_node_id)
		msg.post("@render:", "draw_line", { start_point = from, end_point = to, color = const.COLORS.GREEN })
	end
end

-- Draw a projected path from the mouse position to the goal node or target position
function draw.projected(mouse_position, target_position, projected_path_size, projected_path_entry_point, projected_path_exit_point, projected_path)
	-- Draw line from mouse position to entry point
	msg.post("@render:", "draw_line", { start_point = mouse_position, end_point = projected_path_entry_point, color = const.COLORS.BLUE })

	-- Draw line from entry point to first waypoint
	local first_node = projected_path[1]
	msg.post("@render:", "draw_line", { start_point = projected_path_entry_point, end_point = vmath.vector3(first_node.x, first_node.y, 0), color = const.COLORS.BLUE })

	-- Draw lines between remaining waypoints
	for i = 1, projected_path_size - 1, 1 do
		local from_node = projected_path[i]
		local to_node = projected_path[i + 1]

		msg.post("@render:", "draw_line", { start_point = vmath.vector3(from_node.x, from_node.y, 0), end_point = vmath.vector3(to_node.x, to_node.y, 0), color = const.COLORS.BLUE })
	end

	-- If projected to projected
	if projected_path_exit_point then
		-- Draw line from last node to exit point
		local last_node = projected_path[projected_path_size]
		msg.post("@render:", "draw_line", { start_point = vmath.vector3(last_node.x, last_node.y, 0), end_point = projected_path_exit_point, color = const.COLORS.BLUE })

		-- Draw line from exit point to target position
		msg.post("@render:", "draw_line", { start_point = projected_path_exit_point, end_point = target_position, color = const.COLORS.BLUE })
	end
end

return draw
