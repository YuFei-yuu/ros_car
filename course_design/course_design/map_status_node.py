import os
import threading

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import SaveMap
from std_srvs.srv import Trigger

from course_design.config_utils import load_config_from_node


class MapStatusNode(Node):
    def __init__(self):
        super().__init__('map_status_node')
        self.config, self.config_path = load_config_from_node(self)
        self.mapping_config = self.config.get('mapping', {})

        self.map_topic = self.mapping_config.get('map_topic', '/map')
        self.scan_topic = self.mapping_config.get('scan_topic', '/scan')
        self.save_service_name = self.mapping_config.get(
            'save_service', '/slam_toolbox/save_map')
        self.save_path = self.mapping_config.get(
            'save_path', '/home/ubuntu/ros2_ws/src/slam/maps/map_01')
        self.save_timeout_sec = float(self.mapping_config.get('save_timeout_sec', 15.0))
        self.status_period_sec = float(self.mapping_config.get('status_period_sec', 5.0))

        self.last_map_time = None
        self.last_scan_time = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0
        self.callback_group = ReentrantCallbackGroup()

        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 1)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.save_client = self.create_client(
            SaveMap,
            self.save_service_name,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            '~/save_map',
            self.save_map_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            '~/init_finish',
            self.init_finish_callback,
            callback_group=self.callback_group,
        )
        self.create_timer(self.status_period_sec, self.print_status)

        self.get_logger().info(f'Course mapping status started. config={self.config_path}')
        self.get_logger().info(
            f'Watching map_topic={self.map_topic}, scan_topic={self.scan_topic}, '
            f'save_service={self.save_service_name}'
        )

    def map_callback(self, msg):
        self.last_map_time = self.get_clock().now()
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution

    def scan_callback(self, _msg):
        self.last_scan_time = self.get_clock().now()

    def init_finish_callback(self, _request, response):
        response.success = True
        response.message = 'map_status_node ready'
        return response

    def age_text(self, stamp):
        if stamp is None:
            return 'WAITING'
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return f'OK age={age:.1f}s'

    def print_status(self):
        self.get_logger().info(
            'MAPPING STATUS '
            f'map={self.age_text(self.last_map_time)} '
            f'size={self.map_width}x{self.map_height} '
            f'resolution={self.map_resolution:.3f} '
            f'scan={self.age_text(self.last_scan_time)} '
            f'save_path={self.save_path}'
        )

    def save_map_callback(self, _request, response):
        os.makedirs(os.path.dirname(self.save_path), exist_ok=True)

        if not self.save_client.wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = f'SaveMap service unavailable: {self.save_service_name}'
            self.get_logger().error(response.message)
            return response

        request = SaveMap.Request()
        request.name.data = self.save_path

        self.get_logger().info(f'Saving map as {self.save_path}')
        future = self.save_client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())

        if not done.wait(self.save_timeout_sec):
            response.success = False
            response.message = f'SaveMap timeout after {self.save_timeout_sec:.1f}s'
            self.get_logger().error(response.message)
            return response

        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f'SaveMap failed: {exc}'
            self.get_logger().error(response.message)
            return response

        response.success = result.result == 0
        response.message = f'SaveMap result={result.result} path={self.save_path}'
        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().error(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MapStatusNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
