import threading

import rclpy
from interfaces.srv import SetString
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

from course_design.config_utils import as_bool, load_config_from_node
from course_design.marker_utils import (
    publish_current_goal_marker,
    publish_waypoint_markers,
)
from course_design.navigation_utils import (
    make_stop_publishers,
    publish_stop,
    run_navigation,
    waypoint_to_pose,
)


class BehaviorNode(Node):
    def __init__(self):
        super().__init__('behavior_node')
        self.config, self.config_path = load_config_from_node(self)
        self.navigation_config = self.config.get('navigation', {})
        self.behavior_config = self.config.get('behavior', {})
        self.timeout_sec = float(self.navigation_config.get('timeout_sec', 600.0))
        self.feedback_period_sec = float(
            self.navigation_config.get('feedback_period_sec', 5.0))
        stop_topics = self.navigation_config.get('stop_topics', ['/controller/cmd_vel'])
        self.stop_publishers = make_stop_publishers(self, stop_topics)
        self.callback_group = ReentrantCallbackGroup()
        self.task_lock = threading.Lock()
        self.state = 'IDLE'
        self.autorun_started = False
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoints', 1)
        self.create_timer(5.0, self.publish_markers)

        self.navigator = BasicNavigator()
        self.get_logger().info(f'Behavior config={self.config_path}')
        self.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active.')

        self.create_service(
            SetString,
            '~/run_task',
            self.run_task_callback,
            callback_group=self.callback_group,
        )

        if as_bool(self.behavior_config.get('autorun', True), True):
            delay = float(self.behavior_config.get('start_delay_sec', 2.0))
            self.autorun_timer = self.create_timer(delay, self.autorun_callback)
            self.get_logger().info(
                f'Autorun enabled. task={self.behavior_config.get("default_task", "patrol")}'
            )
        else:
            self.autorun_timer = None
            self.get_logger().info('Autorun disabled. Waiting for ~/run_task.')
        self.publish_markers()

    def publish_markers(self):
        publish_waypoint_markers(self, self.marker_pub, self.config)

    def set_state(self, state, reason=''):
        self.state = state
        if reason:
            self.get_logger().info(f'STATE {state}: {reason}')
        else:
            self.get_logger().info(f'STATE {state}')

    def autorun_callback(self):
        if self.autorun_started:
            return
        self.autorun_started = True
        if self.autorun_timer is not None:
            self.autorun_timer.cancel()
        task = self.behavior_config.get('default_task', 'patrol')
        self.run_task(task)

    def run_task_callback(self, request, response):
        task = request.data.strip() or self.behavior_config.get('default_task', 'patrol')
        success, message = self.run_task(task)
        response.success = success
        response.message = message
        return response

    def run_task(self, task):
        if not self.task_lock.acquire(blocking=False):
            return False, 'behavior task is already running'
        try:
            self.set_state('RUNNING', f'task={task}')
            if task == 'patrol':
                success, message = self.run_patrol()
            elif task == 'return_home':
                success, message = self.go_named_point('home')
            elif task.startswith('go_named_point'):
                waypoint = self.parse_go_named_point(task)
                success, message = self.go_named_point(waypoint)
            elif task in self.config.get('waypoints', {}):
                success, message = self.go_named_point(task)
            else:
                success = False
                message = f'unknown task: {task}'

            if success:
                self.set_state('DONE', message)
            else:
                self.set_state('ERROR', message)
            publish_stop(self.stop_publishers)
            return success, message
        finally:
            self.task_lock.release()

    def parse_go_named_point(self, task):
        if ':' in task:
            return task.split(':', 1)[1].strip()
        parts = task.split()
        if len(parts) >= 2:
            return parts[1].strip()
        return ''

    def go_named_point(self, waypoint_name):
        if not waypoint_name:
            return False, 'waypoint name is empty'
        try:
            pose = waypoint_to_pose(self, self.config, waypoint_name)
        except KeyError as exc:
            self.get_logger().error(str(exc))
            return False, str(exc)

        publish_current_goal_marker(self, self.marker_pub, self.config, waypoint_name)
        success, result = run_navigation(
            self,
            self.navigator,
            pose,
            waypoint_name,
            self.timeout_sec,
            self.feedback_period_sec,
            self.stop_publishers,
        )
        return success, f'{waypoint_name}: {result}'

    def run_patrol(self):
        patrol_points = self.behavior_config.get(
            'patrol_points', ['pick_area', 'goal_green'])
        patrol_count = int(self.behavior_config.get('patrol_count', 3))
        if len(patrol_points) != 2:
            return False, 'patrol_points must contain exactly two waypoint names'

        total_steps = patrol_count * len(patrol_points)
        step = 0
        for patrol_round in range(1, patrol_count + 1):
            for waypoint_name in patrol_points:
                step += 1
                self.get_logger().info(
                    f'PATROL round={patrol_round}/{patrol_count} '
                    f'step={step}/{total_steps} target={waypoint_name}'
                )
                success, message = self.go_named_point(waypoint_name)
                if not success:
                    return (
                        False,
                        f'patrol failed at round={patrol_round} '
                        f'step={step} target={waypoint_name}: {message}',
                    )
        return True, f'patrol completed rounds={patrol_count}'


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        publish_stop(node.stop_publishers)
    finally:
        publish_stop(node.stop_publishers)
        node.navigator.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
