import os
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from interfaces.msg import ColorDetect, ColorsInfo
from interfaces.srv import SetColorDetectParam, SetString
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from servo_controller.action_group_controller import ActionGroupController
from servo_controller_msgs.msg import ServosPosition
from std_srvs.srv import Trigger

from course_design.config_utils import load_config_from_node
from course_design.vision_utils import alignment_command


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place_node')
        self.config, self.config_path = load_config_from_node(self)
        self.transport_config = self.config.get('transport', {})
        self.vision_config = self.config.get('vision', {})
        self.arm_config = self.config.get('arm', {})
        self.target_color = str(self.transport_config.get('target_color', 'red'))
        self.allowed_colors = set(self.transport_config.get('goal_by_color', {}).keys())
        self.allowed_colors.add(self.target_color)
        self.callback_group = ReentrantCallbackGroup()
        self.operation_lock = threading.Lock()
        self.detection_lock = threading.Lock()
        self.abort_event = threading.Event()
        self.latest_detection = None
        self.latest_detection_time = 0.0

        stop_topics = self.config.get('navigation', {}).get(
            'stop_topics', ['/controller/cmd_vel'])
        self.cmd_publishers = [
            self.create_publisher(Twist, topic, 1) for topic in stop_topics
        ]
        self.servo_publisher = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.controller = ActionGroupController(
            self.servo_publisher,
            self.arm_config.get(
                'action_group_dir', '/home/ubuntu/software/arm_pc/ActionGroups'),
        )

        self.create_subscription(
            ColorsInfo,
            '/color_detect/color_info',
            self.color_info_callback,
            10,
            callback_group=self.callback_group,
        )
        self.color_client = self.create_client(
            SetColorDetectParam,
            '/color_detect/set_param',
            callback_group=self.callback_group,
        )
        self.create_service(
            SetString,
            '~/set_target_color',
            self.set_target_color_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            '~/prepare',
            self.prepare_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            '~/pick',
            self.pick_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            '~/place',
            self.place_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            '~/safe',
            self.safe_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            '~/stop',
            self.stop_callback,
            callback_group=self.callback_group,
        )
        self.publish_stop()
        self.get_logger().info(
            f'Pick/place config={self.config_path} target_color={self.target_color}')

    def publish_stop(self):
        message = Twist()
        for publisher in self.cmd_publishers:
            publisher.publish(message)

    def color_info_callback(self, message):
        matches = [item for item in message.data if item.color == self.target_color]
        if not matches:
            return
        # The detector publishes one maximum contour per configured color. Radius is
        # meaningful for circles; it still gives deterministic selection for both types.
        selected = max(matches, key=lambda item: int(item.radius))
        with self.detection_lock:
            self.latest_detection = selected
            self.latest_detection_time = time.monotonic()

    def set_target_color_callback(self, request, response):
        color = request.data.strip().lower()
        if not color:
            response.success = False
            response.message = 'target color is empty'
            return response
        if color not in self.allowed_colors:
            response.success = False
            response.message = f'unsupported target color: {color}'
            return response
        if self.operation_lock.locked():
            response.success = False
            response.message = 'pick/place operation is running'
            return response
        self.target_color = color
        with self.detection_lock:
            self.latest_detection = None
            self.latest_detection_time = 0.0
        response.success = True
        response.message = f'target color set to {color}'
        self.get_logger().info(response.message)
        return response

    def prepare_callback(self, _request, response):
        return self.action_service_callback(
            response, 'pick_ready_action', 'pick_ready_timeout_sec')

    def safe_callback(self, _request, response):
        self.publish_stop()
        return self.action_service_callback(response, 'safe_action', 'safe_timeout_sec')

    def action_service_callback(self, response, action_key, timeout_key):
        if not self.operation_lock.acquire(blocking=False):
            response.success = False
            response.message = 'pick/place operation is already running'
            return response
        try:
            # A preceding stop request must not prevent the arm from returning to its
            # configured safe pose once the active action has exited.
            self.abort_event.clear()
            success, message = self.run_named_action(action_key, timeout_key)
            response.success = success
            response.message = message
            return response
        finally:
            self.publish_stop()
            self.operation_lock.release()

    def pick_callback(self, _request, response):
        if not self.operation_lock.acquire(blocking=False):
            response.success = False
            response.message = 'pick/place operation is already running'
            return response
        try:
            self.abort_event.clear()
            success, message = self.configure_color_detector()
            if success:
                success, message = self.align_target()
            if success:
                success, message = self.run_named_action('pick_action', 'pick_timeout_sec')
            response.success = success
            response.message = message
            return response
        finally:
            self.publish_stop()
            self.operation_lock.release()

    def place_callback(self, _request, response):
        if not self.operation_lock.acquire(blocking=False):
            response.success = False
            response.message = 'pick/place operation is already running'
            return response
        try:
            self.abort_event.clear()
            self.publish_stop()
            success, message = self.run_named_action('place_action', 'place_timeout_sec')
            response.success = success
            response.message = message
            return response
        finally:
            self.publish_stop()
            self.operation_lock.release()

    def stop_callback(self, _request, response):
        self.abort_event.set()
        self.controller.stop_action_group()
        self.publish_stop()
        response.success = True
        response.message = 'stop requested'
        self.get_logger().warn('Pick/place stop requested')
        return response

    def configure_color_detector(self):
        timeout = float(self.transport_config.get('service_timeout_sec', 30.0))
        if not self.color_client.wait_for_service(timeout_sec=timeout):
            return False, '/color_detect/set_param is unavailable'
        request = SetColorDetectParam.Request()
        item = ColorDetect()
        item.color_name = self.target_color
        item.detect_type = str(self.vision_config.get('detect_type', 'rect'))
        request.data = [item]
        future = self.color_client.call_async(request)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            if self.abort_event.is_set():
                return False, 'pick cancelled while configuring detector'
            time.sleep(0.05)
        if not future.done() or future.result() is None:
            return False, 'color detector configuration timed out'
        result = future.result()
        if not result.success:
            return False, f'color detector rejected configuration: {result.message}'
        with self.detection_lock:
            self.latest_detection = None
            self.latest_detection_time = 0.0
        self.get_logger().info(
            f'COLOR CONFIG color={self.target_color} type={item.detect_type}')
        return True, 'color detector configured'

    def current_detection(self):
        with self.detection_lock:
            return self.latest_detection, self.latest_detection_time

    def align_target(self):
        detection_timeout = float(
            self.vision_config.get('detection_timeout_sec', 15.0))
        lost_timeout = float(self.vision_config.get('lost_timeout_sec', 2.0))
        period = max(float(self.vision_config.get('control_period_sec', 0.1)), 0.02)
        stable_required = max(int(self.vision_config.get('stable_frames', 10)), 1)
        started = time.monotonic()
        last_seen = None
        stable_frames = 0
        last_log = 0.0
        self.get_logger().info(f'ALIGN START color={self.target_color}')

        while rclpy.ok() and time.monotonic() - started < detection_timeout:
            if self.abort_event.is_set():
                return False, 'pick cancelled during visual alignment'
            detection, stamp = self.current_detection()
            now = time.monotonic()
            if detection is None or now - stamp > lost_timeout:
                self.publish_stop()
                if last_seen is not None and now - last_seen > lost_timeout:
                    return False, f'target {self.target_color} lost during alignment'
                time.sleep(period)
                continue

            last_seen = now
            command, aligned, error_x, error_y = alignment_command(
                detection, self.vision_config)
            if aligned:
                stable_frames += 1
                self.publish_stop()
                if stable_frames >= stable_required:
                    self.get_logger().info(
                        f'ALIGN DONE color={self.target_color} frames={stable_frames}')
                    return True, f'aligned target {self.target_color}'
            else:
                stable_frames = 0
                for publisher in self.cmd_publishers:
                    publisher.publish(command)
            if now - last_log >= 1.0:
                self.get_logger().info(
                    f'ALIGN color={self.target_color} x={detection.x} y={detection.y} '
                    f'error_x={error_x:.1f} error_y={error_y:.1f} '
                    f'stable={stable_frames}/{stable_required}')
                last_log = now
            time.sleep(period)

        return False, f'target {self.target_color} was not aligned before timeout'

    def run_named_action(self, action_key, timeout_key):
        action_name = str(self.arm_config.get(action_key, '')).strip()
        timeout = float(self.arm_config.get(timeout_key, 15.0))
        action_path = os.path.join(self.controller.action_path, f'{action_name}.d6a')
        if not action_name or not os.path.isfile(action_path):
            return False, f'action group not found: {action_path}'
        if self.operation_lock.locked() and self.abort_event.is_set():
            return False, f'action cancelled: {action_name}'

        finished = threading.Event()

        def run_action():
            try:
                self.controller.run_action(action_name)
            finally:
                finished.set()

        thread = threading.Thread(target=run_action, daemon=True)
        self.get_logger().info(f'ARM START action={action_name} timeout={timeout:.1f}s')
        thread.start()
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not finished.wait(timeout=0.05):
            if self.abort_event.is_set() or time.monotonic() >= deadline:
                self.controller.stop_action_group()
                thread.join(timeout=2.0)
                self.publish_stop()
                if self.abort_event.is_set():
                    return False, f'action cancelled: {action_name}'
                return False, f'action timed out: {action_name}'
        if not finished.is_set():
            return False, f'action interrupted: {action_name}'
        self.get_logger().info(f'ARM DONE action={action_name}')
        return True, f'action completed: {action_name}'


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.abort_event.set()
        node.controller.stop_action_group()
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
