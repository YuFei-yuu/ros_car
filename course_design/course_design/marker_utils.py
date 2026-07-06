from visualization_msgs.msg import Marker, MarkerArray


def waypoint_names(config):
    return sorted((config.get('waypoints') or {}).keys())


def marker_header(node, frame_id):
    header = node.get_clock().now().to_msg()
    return header


def make_marker(frame_id, stamp, namespace, marker_id, marker_type):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    return marker


def set_color(marker, rgba):
    marker.color.r = rgba[0]
    marker.color.g = rgba[1]
    marker.color.b = rgba[2]
    marker.color.a = rgba[3]


def publish_waypoint_markers(node, publisher, config):
    waypoints = config.get('waypoints') or {}
    frame_id = config.get('map_frame', 'map')
    stamp = marker_header(node, frame_id)
    marker_array = MarkerArray()

    colors = [
        (0.2, 0.7, 1.0, 0.95),
        (0.2, 0.9, 0.45, 0.95),
        (1.0, 0.35, 0.25, 0.95),
        (1.0, 0.8, 0.2, 0.95),
        (0.7, 0.45, 1.0, 0.95),
    ]

    for index, name in enumerate(waypoint_names(config)):
        waypoint = waypoints[name]
        x = float(waypoint.get('x', 0.0))
        y = float(waypoint.get('y', 0.0))
        color = colors[index % len(colors)]

        sphere = make_marker(frame_id, stamp, 'course_waypoints', index, Marker.SPHERE)
        sphere.pose.position.x = x
        sphere.pose.position.y = y
        sphere.pose.position.z = 0.06
        sphere.scale.x = 0.16
        sphere.scale.y = 0.16
        sphere.scale.z = 0.12
        set_color(sphere, color)
        marker_array.markers.append(sphere)

        label = make_marker(
            frame_id,
            stamp,
            'course_waypoint_labels',
            index,
            Marker.TEXT_VIEW_FACING,
        )
        label.pose.position.x = x
        label.pose.position.y = y
        label.pose.position.z = 0.32
        label.scale.z = 0.18
        label.text = name
        set_color(label, (1.0, 1.0, 1.0, 0.95))
        marker_array.markers.append(label)

    publisher.publish(marker_array)


def publish_current_goal_marker(node, publisher, config, goal_name):
    waypoints = config.get('waypoints') or {}
    if goal_name not in waypoints:
        return

    waypoint = waypoints[goal_name]
    frame_id = config.get('map_frame', 'map')
    stamp = marker_header(node, frame_id)
    x = float(waypoint.get('x', 0.0))
    y = float(waypoint.get('y', 0.0))
    marker_array = MarkerArray()

    goal = make_marker(frame_id, stamp, 'course_current_goal', 0, Marker.CYLINDER)
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0.08
    goal.scale.x = 0.28
    goal.scale.y = 0.28
    goal.scale.z = 0.16
    set_color(goal, (1.0, 0.85, 0.1, 0.9))
    marker_array.markers.append(goal)

    label = make_marker(frame_id, stamp, 'course_current_goal_label', 1, Marker.TEXT_VIEW_FACING)
    label.pose.position.x = x
    label.pose.position.y = y
    label.pose.position.z = 0.52
    label.scale.z = 0.22
    label.text = f'current: {goal_name}'
    set_color(label, (1.0, 0.95, 0.25, 1.0))
    marker_array.markers.append(label)

    publisher.publish(marker_array)
