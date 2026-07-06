import threading

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from interfaces.srv import SetString
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
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
        self.start_lock = threading.Lock()
        self.state = 'IDLE'
        self.default_task_running = False
        self.initial_pose_received = False
        self.task_thread = None
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoints', 1)
        self.create_timer(5.0, self.publish_markers)

        self.create_service(
            SetString,
            '~/run_task',
            self.run_task_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            '~/start',
            self.start_callback,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Bool,
            self.behavior_config.get('start_topic', '/behavior_start'),
            self.start_topic_callback,
            1,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.behavior_config.get('initial_pose_topic', '/initialpose'),
            self.initial_pose_callback,
            1,
            callback_group=self.callback_group,
        )

        self.require_initial_pose = as_bool(
            self.behavior_config.get('require_initial_pose', True),
            True,
        )
        self.default_task = self.behavior_config.get('default_task', 'patrol')
        if self.require_initial_pose:
            self.set_state('WAIT_INITIAL_POSE', 'set initial pose in RViz, then call ~/start')
        else:
            self.set_state('READY', 'waiting for start command')
        self.get_logger().info(
            'Behavior is armed but not running. '
            'Use RViz 2D Pose Estimate first, then run: '
            'ros2 service call /behavior_node/start std_srvs/srv/Trigger "{}"'
        )
        self.publish_markers()

        self.navigator = BasicNavigator()
        self.get_logger().info(f'Behavior config={self.config_path}')
        self.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active.')

    def publish_markers(self):
        publish_waypoint_markers(self, self.marker_pub, self.config)

    def set_state(self, state, reason=''):
        self.state = state
        if reason:
            self.get_logger().info(f'STATE {state}: {reason}')
        else:
            self.get_logger().info(f'STATE {state}')

    def initial_pose_callback(self, msg):
        self.initial_pose_received = True
        self.get_logger().info(
            'Initial pose received from RViz: '
            f'x={msg.pose.pose.position.x:.3f}, '
            f'y={msg.pose.pose.position.y:.3f}'
        )
        if self.state == 'WAIT_INITIAL_POSE':
            self.set_state('READY', 'initial pose received, waiting for start command')

    def start_callback(self, _request, response):
        success, message = self.start_default_task_async()
        response.success = success
        response.message = message
        return response

    def start_topic_callback(self, msg):
        if not msg.data:
            self.get_logger().info('Ignoring /behavior_start false')
            return
        success, message = self.start_default_task_async()
        if not success:
            self.get_logger().error(f'Start request rejected: {message}')

    def start_default_task_async(self):
        with self.start_lock:
            if self.default_task_running:
                return False, 'default behavior is already running'
            if self.task_lock.locked():
                return False, 'another behavior task is already running'
            if self.require_initial_pose and not self.initial_pose_received:
                self.set_state('WAIT_INITIAL_POSE', 'start rejected: no /initialpose yet')
                return False, 'please set initial pose in RViz with 2D Pose Estimate first'
            self.default_task_running = True

        self.get_logger().info(
            f'Start command accepted. task={self.default_task} running in background'
        )
        self.task_thread = threading.Thread(
            target=self.run_default_task,
            args=(self.default_task,),
            daemon=True,
        )
        self.task_thread.start()
        return True, f'started default task: {self.default_task}'

    def run_default_task(self, task):
        try:
            self.run_task(task)
        finally:
            with self.start_lock:
                self.default_task_running = False
            if self.state in ('DONE', 'ERROR'):
                self.get_logger().info('Default behavior finished. Ready for next start command.')

    def run_task_callback(self, request, response):
        task = request.data.strip() or self.behavior_config.get('default_task', 'patrol')
        if self.require_initial_pose and not self.initial_pose_received:
            response.success = False
            response.message = 'please set initial pose in RViz with 2D Pose Estimate first'
            self.set_state('WAIT_INITIAL_POSE', 'run_task rejected: no /initialpose yet')
            return response
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
