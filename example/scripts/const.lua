local const            = {}

const.EPSILON          = 0.0001
const.PLANE_POINT      = vmath.vector3(0, 0, 0)
const.PLANE_NORMAL     = vmath.vector3(0, 1, 0)

const.FACTORIES        = {
	AGENT     = "/factories#agent",
	DIRECTION = "/factories#direction",
	NODE      = "/factories#node"
}

const.COLORS           = {
	RED   = vmath.vector4(1, 0, 0, 1),
	GREEN = vmath.vector4(0, 1, 0, 1),
	BLUE  = vmath.vector4(0, 0, 1, 1)
}

const.AGENT_STATES     = {
	INACTIVE       = 0, -- Not in navigation system
	ACTIVE         = 1, -- Following path
	PAUSED         = 2, -- Paused by application
	REPLANNING     = 3, -- Detected invalidation, finding new path
	ARRIVED        = 4, -- Reached goal,
	WAITTING_ORDER = 5
}

const.SMOOTHING_CONFIG = {
	style                               = pathfinder.PathSmoothStyle.BEZIER_QUADRATIC,
	bezier_sample_segment               = 8, -- Number of segments per curve
	bezier_control_point_offset         = 0.4, -- For bezier_cubic style
	bezier_curve_radius                 = 0.9, -- For bezier_quadratic style (active)
	bezier_adaptive_tightness           = 0.2, -- For bezier_adaptive style
	bezier_adaptive_roundness           = 0.2, -- For bezier_adaptive style
	bezier_adaptive_max_corner_distance = 50.0, -- For bezier_adaptive style
	bezier_arc_radius                   = 0.3, -- For circular_arc style
}

const.TRIGGERS         =
{
	KEY_1              = hash("KEY_1"),
	MOUSE_BUTTON_1     = hash("MOUSE_BUTTON_1"),
	MOUSE_WHEEL_UP     = hash("MOUSE_WHEEL_UP"),
	MOUSE_WHEEL_DOWN   = hash("MOUSE_WHEEL_DOWN"),
	MOUSE_BUTTON_2     = hash("MOUSE_BUTTON_2"),
	MOUSE_BUTTON_RIGHT = hash("MOUSE_BUTTON_RIGHT"),
}
return const
