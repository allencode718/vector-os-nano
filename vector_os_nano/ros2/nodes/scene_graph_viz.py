"""RViz MarkerArray publisher for the three-layer scene graph.

Publishes to /scene_graph_markers (visualization_msgs/MarkerArray) at 1 Hz.
Designed to be embedded in the Go2VNavBridge node or run standalone.

Marker types:
    - Room boundaries: LineStrip (colored per room)
    - Room labels: Text at room centers (with visit count + coverage)
    - Viewpoints: Sphere (green) at observation positions
    - Objects: Cube (orange) with text labels
    - Robot trail: Sphere (teal) at current position
    - Navigation goal: Arrow (red)

All markers are in the "map" frame.
"""
from __future__ import annotations

import math
import time
from typing import Any

# Lazy ROS2 imports — this module is only loaded when ROS2 is available.
_rclpy = None
_MarkerArray = None
_Marker = None
_ColorRGBA = None
_Point = None
_Vector3 = None
_Header = None


def _ensure_imports() -> bool:
    """Lazy-import ROS2 message types. Returns True if available."""
    global _rclpy, _MarkerArray, _Marker, _ColorRGBA, _Point, _Vector3, _Header
    if _MarkerArray is not None:
        return True
    try:
        import rclpy as _r
        from visualization_msgs.msg import Marker, MarkerArray
        from std_msgs.msg import ColorRGBA, Header
        from geometry_msgs.msg import Point, Vector3
        _rclpy = _r
        _MarkerArray = MarkerArray
        _Marker = Marker
        _ColorRGBA = ColorRGBA
        _Point = Point
        _Vector3 = Vector3
        _Header = Header
        return True
    except ImportError:
        return False


# Room boundaries (matches go2_room.xml)
_ROOM_BOUNDS: dict[str, tuple[float, float, float, float]] = {
    "living_room":    (0.0,  0.0,  6.0,  5.0),
    "dining_room":    (0.0,  5.0,  6.0,  10.0),
    "kitchen":        (14.0, 0.0,  20.0, 5.0),
    "study":          (14.0, 5.0,  20.0, 10.0),
    "master_bedroom": (0.0,  10.0, 7.0,  14.0),
    "guest_bedroom":  (12.0, 10.0, 20.0, 14.0),
    "bathroom":       (7.0,  10.0, 10.0, 14.0),
    "hallway":        (6.0,  0.0,  14.0, 10.0),
}

_ROOM_CENTERS: dict[str, tuple[float, float]] = {
    "living_room":    (3.0,  2.5),
    "dining_room":    (3.0,  7.5),
    "kitchen":        (17.0, 2.5),
    "study":          (17.0, 7.5),
    "master_bedroom": (3.5,  12.0),
    "guest_bedroom":  (16.0, 12.0),
    "bathroom":       (8.5,  12.0),
    "hallway":        (10.0, 5.0),
}

# RGBA colors per room (r, g, b, a) in [0, 1]
_ROOM_COLORS: dict[str, tuple[float, float, float, float]] = {
    "living_room":    (0.27, 0.51, 0.71, 0.3),
    "dining_room":    (0.80, 0.52, 0.25, 0.3),
    "kitchen":        (0.24, 0.70, 0.44, 0.3),
    "study":          (0.58, 0.44, 0.86, 0.3),
    "master_bedroom": (0.86, 0.44, 0.58, 0.3),
    "guest_bedroom":  (1.00, 0.65, 0.00, 0.3),
    "bathroom":       (0.00, 0.81, 0.82, 0.3),
    "hallway":        (0.66, 0.66, 0.66, 0.2),
}


def build_scene_graph_markers(
    scene_graph: Any,
    stamp: Any = None,
    robot_x: float = 0.0,
    robot_y: float = 0.0,
    robot_heading: float = 0.0,
    nav_goal: tuple[float, float] | None = None,
) -> Any:
    """Build a MarkerArray from the current scene graph state.

    Args:
        scene_graph: SceneGraph instance (or None for static room layout only).
        stamp: ROS2 Time stamp. If None, uses current time.
        robot_x, robot_y: Current robot position.
        robot_heading: Current heading in radians.
        nav_goal: Optional (x, y) of current navigation target.

    Returns:
        visualization_msgs/MarkerArray, or None if ROS2 not available.
    """
    if not _ensure_imports():
        return None

    markers = _MarkerArray()
    marker_id = 0

    if stamp is None:
        import builtin_interfaces.msg
        now = time.time()
        stamp = builtin_interfaces.msg.Time()
        stamp.sec = int(now)
        stamp.nanosec = int((now % 1) * 1e9)

    header = _Header()
    header.frame_id = "map"
    header.stamp = stamp

    # --- Room boundaries (static, always shown) ---
    for room_name, (x0, y0, x1, y1) in _ROOM_BOUNDS.items():
        color = _ROOM_COLORS.get(room_name, (0.5, 0.5, 0.5, 0.3))

        # Filled rectangle — color and opacity reflect exploration status:
        #   unvisited: grey, alpha=0.15
        #   visited:   room color, alpha scales with coverage (0.2 → 0.6)
        is_visited = False
        coverage = 0.0
        if scene_graph is not None:
            room_node = scene_graph.get_room(room_name)
            if room_node and room_node.visit_count > 0:
                is_visited = True
                coverage = scene_graph.get_room_coverage(room_name)

        m = _Marker()
        m.header = header
        m.ns = "rooms"
        m.id = marker_id
        marker_id += 1
        m.type = _Marker.CUBE
        m.action = _Marker.ADD
        m.pose.position.x = (x0 + x1) / 2
        m.pose.position.y = (y0 + y1) / 2
        m.pose.position.z = 0.01
        m.scale.x = x1 - x0
        m.scale.y = y1 - y0
        m.scale.z = 0.02
        if is_visited:
            m.color.r = color[0]
            m.color.g = color[1]
            m.color.b = color[2]
            m.color.a = 0.2 + coverage * 0.4  # 0.2 (just visited) → 0.6 (fully covered)
        else:
            m.color.r = 0.4
            m.color.g = 0.4
            m.color.b = 0.4
            m.color.a = 0.15
        m.lifetime.sec = 0  # persistent
        markers.markers.append(m)

        # Room label
        m_text = _Marker()
        m_text.header = header
        m_text.ns = "room_labels"
        m_text.id = marker_id
        marker_id += 1
        m_text.type = _Marker.TEXT_VIEW_FACING
        m_text.action = _Marker.ADD
        cx, cy = _ROOM_CENTERS[room_name]
        m_text.pose.position.x = cx
        m_text.pose.position.y = cy
        m_text.pose.position.z = 0.5

        # Label text: room name + visit info from scene graph
        label = room_name
        if scene_graph is not None:
            room_node = scene_graph.get_room(room_name)
            if room_node and room_node.visit_count > 0:
                cov = scene_graph.get_room_coverage(room_name)
                n_objs = len(scene_graph.find_objects_in_room(room_name))
                label = f"{room_name}\n{room_node.visit_count}x | {cov:.0%} | {n_objs} obj"

        m_text.text = label
        m_text.scale.z = 0.4  # text height
        m_text.color.r = 1.0
        m_text.color.g = 1.0
        m_text.color.b = 1.0
        m_text.color.a = 0.9
        markers.markers.append(m_text)

    # --- Viewpoints (green spheres) ---
    if scene_graph is not None:
        for room in scene_graph.get_all_rooms():
            for vp in scene_graph.get_viewpoints_in_room(room.room_id):
                m = _Marker()
                m.header = header
                m.ns = "viewpoints"
                m.id = marker_id
                marker_id += 1
                m.type = _Marker.SPHERE
                m.action = _Marker.ADD
                m.pose.position.x = vp.x
                m.pose.position.y = vp.y
                m.pose.position.z = 0.15
                m.scale.x = 0.3
                m.scale.y = 0.3
                m.scale.z = 0.3
                m.color.r = 0.0
                m.color.g = 0.8
                m.color.b = 0.0
                m.color.a = 0.7
                markers.markers.append(m)

    # --- Objects (orange cubes with labels) ---
    if scene_graph is not None:
        for room in scene_graph.get_all_rooms():
            objs = scene_graph.find_objects_in_room(room.room_id)
            cx, cy = room.center_x, room.center_y
            for i, obj in enumerate(objs):
                angle = i * 2 * math.pi / max(len(objs), 1)
                ox = cx + 0.8 * math.cos(angle)
                oy = cy + 0.8 * math.sin(angle)

                # Object cube
                m = _Marker()
                m.header = header
                m.ns = "objects"
                m.id = marker_id
                marker_id += 1
                m.type = _Marker.CUBE
                m.action = _Marker.ADD
                m.pose.position.x = ox
                m.pose.position.y = oy
                m.pose.position.z = 0.15
                m.scale.x = 0.25
                m.scale.y = 0.25
                m.scale.z = 0.25
                m.color.r = 1.0
                m.color.g = 0.55
                m.color.b = 0.0
                m.color.a = 0.8
                markers.markers.append(m)

                # Object label
                m_label = _Marker()
                m_label.header = header
                m_label.ns = "object_labels"
                m_label.id = marker_id
                marker_id += 1
                m_label.type = _Marker.TEXT_VIEW_FACING
                m_label.action = _Marker.ADD
                m_label.pose.position.x = ox
                m_label.pose.position.y = oy
                m_label.pose.position.z = 0.45
                m_label.text = obj.category
                m_label.scale.z = 0.25
                m_label.color.r = 1.0
                m_label.color.g = 0.8
                m_label.color.b = 0.0
                m_label.color.a = 0.9
                markers.markers.append(m_label)

    # --- Robot position (teal arrow) ---
    m_robot = _Marker()
    m_robot.header = header
    m_robot.ns = "robot"
    m_robot.id = marker_id
    marker_id += 1
    m_robot.type = _Marker.ARROW
    m_robot.action = _Marker.ADD
    m_robot.pose.position.x = robot_x
    m_robot.pose.position.y = robot_y
    m_robot.pose.position.z = 0.2
    # Quaternion from heading
    m_robot.pose.orientation.z = math.sin(robot_heading / 2)
    m_robot.pose.orientation.w = math.cos(robot_heading / 2)
    m_robot.scale.x = 0.6  # arrow length
    m_robot.scale.y = 0.15  # arrow width
    m_robot.scale.z = 0.15  # arrow height
    m_robot.color.r = 0.0
    m_robot.color.g = 0.71
    m_robot.color.b = 0.71
    m_robot.color.a = 1.0
    markers.markers.append(m_robot)

    # --- Navigation goal (red sphere) ---
    if nav_goal is not None:
        m_goal = _Marker()
        m_goal.header = header
        m_goal.ns = "nav_goal"
        m_goal.id = marker_id
        marker_id += 1
        m_goal.type = _Marker.SPHERE
        m_goal.action = _Marker.ADD
        m_goal.pose.position.x = nav_goal[0]
        m_goal.pose.position.y = nav_goal[1]
        m_goal.pose.position.z = 0.3
        m_goal.scale.x = 0.5
        m_goal.scale.y = 0.5
        m_goal.scale.z = 0.5
        m_goal.color.r = 1.0
        m_goal.color.g = 0.1
        m_goal.color.b = 0.1
        m_goal.color.a = 0.8
        markers.markers.append(m_goal)

    return markers
