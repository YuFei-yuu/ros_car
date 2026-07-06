import threading

import rclpy
from interfaces.srv import SetString
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray

from course_design.config_utils import load_config_from_node
from course_design.marker_utils import (
    publish_current_goal_marker,
    publish_waypoint_markers,
)
from course_design.navigation_utils import (
    make_stop_publishers,
    run_navigation,
    waypoint_to_pose,
)


class WaypointNavNode(Node):
    FIXED_SERVICES = {
        'home': 'go_home',
        'pick_area': 'go_pick_area',
        'goal_red': 'go_goal_red',
        'goal_green': 'go_goal_green',
        'goal_blue': 'go_goal_blue',
    }

    def __init__(self):
        super().__init__('waypoint_nav_node')
        self.config, self.config_path = load_config_from_node(self)
        self.navigation_config = self.config.get('navigation', {})
        self.timeout_sec = float(self.navigation_config.get('timeout_sec', 600.0))
        self.feedback_period_sec = float(
            self.navigation_config.get('feedback_period_sec', 5.0))
        stop_topics = self.navigation_config.get('stop_topics', ['/controller/cmd_vel'])
        self.stop_publishers = make_stop_publishers(self, stop_topics)
        self.callback_group = ReentrantCallbackGroup()
        self.nav_lock = threading.Lock()
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoints', 1)
        self.create_timer(5.0, self.publish_markers)

        self.navigator = BasicNavigator()
        self.get_logger().info(f'Waypoint navigation config={self.config_path}')
        self.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active.')

        for waypoint_name, service_name in self.FIXED_SERVICES.items():
            self.create_service(
                Trigger,
                f'~/{service_name}',
                self.make_trigger_callback(waypoint_name),
                callback_group=self.callback_group,
            )
        self.create_service(
            SetString,
            '~/go_named_point',
            self.go_named_point_callback,
            callback_group=self.callback_group,
        )
        self.create_service(Trigger, '~/init_finish', self.init_finish_callback)
        self.get_logger().info('Waypoint navigation services are ready.')
        self.publish_markers()

    def init_finish_callback(self, _request, response):
        response.success = True
        response.message = 'waypoint_nav_node ready'
        return response

    def publish_markers(self):
        publish_waypoint_markers(self, self.marker_pub, self.config)

    def make_trigger_callback(self, waypoint_name):
        def callback(_request, response):
            success, message = self.navigate_to_waypoint(waypoint_name)
            response.success = success
            response.message = message
            return response
        return callback

    def go_named_point_callback(self, request, response):
        waypoint_name = request.data.strip()
        success, message = self.navigate_to_waypoint(waypoint_name)
        response.success = success
        response.message = message
        return response

    def navigate_to_waypoint(self, waypoint_name):
        if not waypoint_name:
            return False, 'waypoint name is empty'
        if not self.nav_lock.acquire(blocking=False):
            return False, 'navigation is already running'

        try:
            pose = waypoint_to_pose(self, self.config, waypoint_name)
            publish_current_goal_marker(
                self,
                self.marker_pub,
                self.config,
                waypoint_name,
            )
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
        except KeyError as exc:
            self.get_logger().error(str(exc))
            return False, str(exc)
        finally:
            self.nav_lock.release()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.navigator.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
