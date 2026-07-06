import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.duration import Duration


def yaw_to_quaternion(yaw):
    quaternion = Quaternion()
    quaternion.z = math.sin(yaw * 0.5)
    quaternion.w = math.cos(yaw * 0.5)
    return quaternion


def waypoint_to_pose(node, config, name):
    waypoints = config.get('waypoints', {})
    if name not in waypoints:
        available = ', '.join(sorted(waypoints.keys()))
        raise KeyError(f'Unknown waypoint "{name}". Available: {available}')

    waypoint = waypoints[name]
    yaw = waypoint.get('yaw')
    if yaw is None:
        yaw = math.radians(float(waypoint.get('yaw_deg', 0.0)))

    pose = PoseStamped()
    pose.header.frame_id = config.get('map_frame', 'map')
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = float(waypoint.get('x', 0.0))
    pose.pose.position.y = float(waypoint.get('y', 0.0))
    pose.pose.orientation = yaw_to_quaternion(float(yaw))
    return pose


def describe_pose(pose):
    return (
        f'x={pose.pose.position.x:.3f}, '
        f'y={pose.pose.position.y:.3f}, '
        f'frame={pose.header.frame_id}'
    )


def duration_seconds(duration_msg):
    if duration_msg is None:
        return None
    return Duration.from_msg(duration_msg).nanoseconds / 1e9


def task_result_label(result):
    if result == TaskResult.SUCCEEDED:
        return 'SUCCEEDED'
    if result == TaskResult.CANCELED:
        return 'CANCELED'
    if result == TaskResult.FAILED:
        return 'FAILED'
    if result is None:
        return 'UNKNOWN'
    return str(result)


def make_stop_publishers(node, topics):
    publishers = []
    for topic in topics:
        publishers.append(node.create_publisher(Twist, topic, 1))
    return publishers


def publish_stop(publishers):
    twist = Twist()
    for publisher in publishers:
        publisher.publish(twist)


def run_navigation(
    node,
    navigator,
    pose,
    goal_name,
    timeout_sec,
    feedback_period_sec,
    stop_publishers=None,
):
    timeout_sec = float(timeout_sec)
    feedback_period_sec = max(float(feedback_period_sec), 0.5)
    stop_publishers = stop_publishers or []

    node.get_logger().info(f'NAV START goal={goal_name} {describe_pose(pose)}')
    navigator.goToPose(pose)

    started = time.monotonic()
    last_feedback = 0.0
    timed_out = False

    while rclpy.ok() and not navigator.isTaskComplete():
        now = time.monotonic()
        feedback = navigator.getFeedback()
        if now - last_feedback >= feedback_period_sec:
            if feedback is not None:
                nav_time = duration_seconds(feedback.navigation_time)
                eta = duration_seconds(feedback.estimated_time_remaining)
                node.get_logger().info(
                    f'NAV FEEDBACK goal={goal_name} '
                    f'time={nav_time:.1f}s eta={eta:.1f}s'
                )
            else:
                node.get_logger().info(f'NAV FEEDBACK goal={goal_name} waiting')
            last_feedback = now

        if now - started > timeout_sec:
            timed_out = True
            node.get_logger().error(
                f'NAV TIMEOUT goal={goal_name} timeout={timeout_sec:.1f}s'
            )
            navigator.cancelTask()
            break

        time.sleep(0.1)

    if timed_out:
        cancel_deadline = time.monotonic() + 3.0
        while rclpy.ok() and not navigator.isTaskComplete():
            if time.monotonic() >= cancel_deadline:
                break
            time.sleep(0.1)

    result = navigator.getResult() if navigator.isTaskComplete() else None
    label = 'TIMEOUT' if timed_out else task_result_label(result)
    success = result == TaskResult.SUCCEEDED and not timed_out

    if success:
        node.get_logger().info(f'NAV RESULT goal={goal_name} result={label}')
    else:
        node.get_logger().error(f'NAV RESULT goal={goal_name} result={label}')

    publish_stop(stop_publishers)
    return success, label
