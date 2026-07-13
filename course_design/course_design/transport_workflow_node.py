import os
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from interfaces.srv import SetString
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from course_design.config_utils import as_bool, load_config_from_node
from course_design.navigation_utils import (
    make_stop_publishers,
    publish_stop,
    run_navigation,
    waypoint_to_pose,
)


class TransportWorkflowNode(Node):
    def __init__(self):
        super().__init__('transport_workflow_node')
        self.config, self.config_path = load_config_from_node(self)
        self.transport_config = self.config.get('transport', {})
        self.navigation_config = self.config.get('navigation', {})
        self.arm_config = self.config.get('arm', {})
        self.callback_group = ReentrantCallbackGroup()
        self.workflow_lock = threading.Lock()
        self.cancel_event = threading.Event()
        self.initial_pose_received = False
        self.state = 'WAIT_INITIAL_POSE'
        self.color = str(self.transport_config.get('target_color', 'red')).lower()
        self.goal_name = ''
        self.phase = ''

        stop_topics = self.navigation_config.get('stop_topics', ['/controller/cmd_vel'])
        self.stop_publishers = make_stop_publishers(self, stop_topics)
        self.state_publisher = self.create_publisher(
            String, self.transport_config.get('state_topic', '/transport_workflow/state'), 10)
        self.pick_place_node = self.transport_config.get(
            'pick_place_node', '/pick_place_node').rstrip('/')
        self.set_color_client = self.create_client(
            SetString, f'{self.pick_place_node}/set_target_color',
            callback_group=self.callback_group)
        self.prepare_client = self.create_client(
            Trigger, f'{self.pick_place_node}/prepare',
            callback_group=self.callback_group)
        self.pick_client = self.create_client(
            Trigger, f'{self.pick_place_node}/pick',
            callback_group=self.callback_group)
        self.place_client = self.create_client(
            Trigger, f'{self.pick_place_node}/place',
            callback_group=self.callback_group)
        self.safe_client = self.create_client(
            Trigger, f'{self.pick_place_node}/safe',
            callback_group=self.callback_group)
        self.stop_client = self.create_client(
            Trigger, f'{self.pick_place_node}/stop',
            callback_group=self.callback_group)

        self.create_service(
            Trigger, '~/start', self.start_callback, callback_group=self.callback_group)
        self.create_service(
            Trigger, '~/cancel', self.cancel_callback, callback_group=self.callback_group)
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.transport_config.get('initial_pose_topic', '/initialpose'),
            self.initial_pose_callback,
            1,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Bool, self.transport_config.get('start_topic', '/transport_start'),
            self.start_topic_callback, 1, callback_group=self.callback_group)
        self.create_subscription(
            Bool, self.transport_config.get('cancel_topic', '/transport_cancel'),
            self.cancel_topic_callback, 1, callback_group=self.callback_group)

        self.navigator = BasicNavigator()
        self.get_logger().info(f'Transport config={self.config_path}')
        self.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active.')
        if not self.requires_initial_pose():
            self.set_state('READY', 'waiting for start command')
        else:
            self.set_state('WAIT_INITIAL_POSE', 'set initial pose in RViz before start')

    def requires_initial_pose(self):
        return as_bool(self.transport_config.get('require_initial_pose', True), True)

    def set_state(self, state, reason=''):
        self.state = state
        message = String()
        message.data = state
        self.state_publisher.publish(message)
        detail = f'STATE {state}'
        if reason:
            detail = f'{detail}: {reason}'
        self.get_logger().info(detail)

    def initial_pose_callback(self, message):
        self.initial_pose_received = True
        self.get_logger().info(
            'Initial pose received: '
            f'x={message.pose.pose.position.x:.3f} y={message.pose.pose.position.y:.3f}')
        if self.state == 'WAIT_INITIAL_POSE':
            self.set_state('READY', 'initial pose received, waiting for start')

    def start_callback(self, _request, response):
        success, message = self.start_async()
        response.success = success
        response.message = message
        return response

    def start_topic_callback(self, message):
        if message.data:
            success, reason = self.start_async()
            if not success:
                self.get_logger().error(f'Transport start rejected: {reason}')

    def cancel_callback(self, _request, response):
        self.request_cancel()
        response.success = True
        response.message = 'transport cancellation requested'
        return response

    def cancel_topic_callback(self, message):
        if message.data:
            self.request_cancel()

    def start_async(self):
        if self.requires_initial_pose() and not self.initial_pose_received:
            self.set_state('WAIT_INITIAL_POSE', 'start rejected: no initial pose')
            return False, 'set initial pose in RViz first'
        if self.workflow_lock.locked():
            return False, 'transport workflow is already running'
        self.cancel_event.clear()
        thread = threading.Thread(target=self.run_workflow, daemon=True)
        thread.start()
        return True, 'transport workflow started'

    def request_cancel(self):
        self.cancel_event.set()
        try:
            self.navigator.cancelTask()
        except Exception as exc:
            self.get_logger().warn(f'Nav2 cancel request failed: {exc}')
        self.call_async(self.stop_client, Trigger.Request(), 'pick/place stop', quiet=True)
        publish_stop(self.stop_publishers)
        self.get_logger().warn('Transport cancellation requested')

    def run_workflow(self):
        if not self.workflow_lock.acquire(blocking=False):
            return
        try:
            success, reason = self.preflight()
            if success:
                self.set_state('INITIALIZE', f'color={self.color} goal={self.goal_name}')
                success, reason = self.call_service(
                    self.prepare_client, Trigger.Request(), 'arm initialization')
            if success:
                success, reason = self.navigate('GO_PICK_AREA', 'pick_area')
            if success:
                self.set_state('PICK', f'color={self.color}')
                success, reason = self.call_service(
                    self.set_color_client, self.color_request(), 'set target color')
            if success:
                success, reason = self.call_service(
                    self.pick_client, Trigger.Request(), 'pick target')
            if success:
                success, reason = self.navigate('GO_GOAL', self.goal_name)
            if success:
                self.set_state('PLACE', f'goal={self.goal_name}')
                publish_stop(self.stop_publishers)
                success, reason = self.call_service(
                    self.place_client, Trigger.Request(), 'place target')
            if success:
                success, reason = self.navigate('RETURN_HOME', 'home')

            if success:
                self.set_state('SAFE', 'returning arm to safe pose')
                publish_stop(self.stop_publishers)
                success, reason = self.call_service(
                    self.safe_client, Trigger.Request(), 'safe arm pose')
            if success:
                self.set_state('DONE', f'color={self.color} returned home')
                publish_stop(self.stop_publishers)
            else:
                final_state = 'CANCELED' if self.cancel_event.is_set() else 'ERROR'
                self.set_state(final_state, self.failure_reason(reason))
                publish_stop(self.stop_publishers)
                self.call_async(self.safe_client, Trigger.Request(), 'safe arm pose', quiet=True)
        finally:
            publish_stop(self.stop_publishers)
            self.workflow_lock.release()

    def preflight(self):
        self.color = str(self.transport_config.get('target_color', 'red')).lower()
        goal_by_color = self.transport_config.get('goal_by_color', {})
        self.goal_name = str(goal_by_color.get(self.color, '')).strip()
        if not self.color or not self.goal_name:
            return False, f'no goal configured for target color {self.color!r}'
        waypoints = self.config.get('waypoints', {})
        for name in ('home', 'pick_area', self.goal_name):
            if name not in waypoints:
                return False, f'required waypoint is not configured: {name}'
        action_dir = str(self.arm_config.get('action_group_dir', '')).strip()
        for key in ('pick_ready_action', 'pick_action', 'place_action', 'safe_action'):
            action = str(self.arm_config.get(key, '')).strip()
            path = os.path.join(action_dir, f'{action}.d6a')
            if not action or not os.path.isfile(path):
                return False, f'action group not found: {path}'
        for client, name in (
            (self.set_color_client, 'set_target_color'),
            (self.prepare_client, 'prepare'),
            (self.pick_client, 'pick'),
            (self.place_client, 'place'),
            (self.safe_client, 'safe'),
        ):
            if not client.wait_for_service(
                    timeout_sec=float(self.transport_config.get('service_timeout_sec', 30.0))):
                return False, f'pick/place service unavailable: {name}'
        if self.cancel_event.is_set():
            return False, 'cancelled before initialization'
        return True, 'preflight passed'

    def color_request(self):
        request = SetString.Request()
        request.data = self.color
        return request

    def navigate(self, state, waypoint):
        if self.cancel_event.is_set():
            return False, 'cancelled before navigation'
        self.set_state(state, f'goal={waypoint}')
        try:
            pose = waypoint_to_pose(self, self.config, waypoint)
        except KeyError as exc:
            return False, str(exc)
        success, result = run_navigation(
            self,
            self.navigator,
            pose,
            waypoint,
            self.navigation_config.get('timeout_sec', 600.0),
            self.navigation_config.get('feedback_period_sec', 5.0),
            self.stop_publishers,
        )
        if self.cancel_event.is_set():
            return False, f'cancelled during navigation to {waypoint}'
        return success, f'navigation {waypoint}: {result}'

    def call_service(self, client, request, label):
        if self.cancel_event.is_set():
            return False, f'cancelled before {label}'
        timeout = float(self.transport_config.get('service_timeout_sec', 30.0))
        if not client.wait_for_service(timeout_sec=timeout):
            return False, f'service unavailable for {label}'
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            if self.cancel_event.is_set():
                return False, f'cancelled during {label}'
            time.sleep(0.05)
        if not future.done() or future.result() is None:
            return False, f'service timed out for {label}'
        response = future.result()
        if not response.success:
            return False, f'{label} failed: {response.message}'
        self.get_logger().info(f'WORKFLOW {label}: {response.message}')
        return True, response.message

    def call_async(self, client, request, label, quiet):
        if not client.service_is_ready():
            if not quiet:
                self.get_logger().warn(f'service unavailable for {label}')
            return
        future = client.call_async(request)

        def report_result(done_future):
            try:
                response = done_future.result()
                if response is not None and not response.success and not quiet:
                    self.get_logger().warn(f'{label} failed: {response.message}')
            except Exception as exc:
                if not quiet:
                    self.get_logger().warn(f'{label} failed: {exc}')

        future.add_done_callback(report_result)

    def failure_reason(self, reason):
        return f'phase={self.state} color={self.color} goal={self.goal_name} reason={reason}'


def main(args=None):
    rclpy.init(args=args)
    node = TransportWorkflowNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.request_cancel()
        node.navigator.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
