import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from interfaces.msg import ObjectsInfo
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray

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
from yolo_patrol.config_utils import as_bool, load_config_from_node, load_yaml
from yolo_patrol.detection_filter import DetectionWindow, MovingWatchBuffer


class YoloPatrolNode(Node):
    def __init__(self):
        super().__init__('yolo_patrol_node')
        self.config, self.config_path = load_config_from_node(self)
        self.course_config, self.course_config_path = self.load_course_config()

        self.nav_config = self.config.get('navigation', {})
        self.task_config = self.config.get('task', {})
        self.yolo_config = self.config.get('yolo', {})
        self.det_config = self.config.get('detection', {})
        self.policy_config = self.config.get('policy', {})
        self.report_config = self.config.get('report', {})

        self.timeout_sec = float(self.nav_config.get('timeout_sec', 300.0))
        self.feedback_period_sec = float(self.nav_config.get('feedback_period_sec', 3.0))
        stop_topics = self.nav_config.get('stop_topics', ['/controller/cmd_vel', '/cmd_vel'])
        self.stop_publishers = make_stop_publishers(self, stop_topics)

        self.watch_classes = self.det_config.get('watch_classes', [])
        self.inspect_lock = threading.RLock()
        self.active_window = None
        self.inspect_active = False
        self.last_yolo_msg_time = 0.0
        self.total_yolo_msg_count = 0
        self.moving_watch = MovingWatchBuffer(
            self.watch_classes,
            self.det_config.get('moving_min_score', 0.7),
            self.det_config.get('moving_candidate_max_age_sec', 15.0),
        )

        self.state = 'IDLE'
        self.initial_pose_received = False
        self.running = False
        self.reset_event = threading.Event()
        self.task_thread = None
        self.task_lock = threading.Lock()
        self.callback_group = ReentrantCallbackGroup()

        self.status_pub = self.create_publisher(
            String,
            self.report_config.get('status_topic', '/yolo_patrol/status'),
            10,
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoints', 1)
        self.create_timer(5.0, self.publish_markers)
        self.create_subscription(
            ObjectsInfo,
            self.yolo_config.get('object_topic', '/yolo/object_detect'),
            self.object_callback,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.task_config.get('initial_pose_topic', '/initialpose'),
            self.initial_pose_callback,
            1,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Bool,
            self.task_config.get('start_topic', '/yolo_patrol/start'),
            self.start_topic_callback,
            1,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Bool,
            self.task_config.get('reset_topic', '/yolo_patrol/reset'),
            self.reset_topic_callback,
            1,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            '~/start',
            self.start_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            '~/reset',
            self.reset_callback,
            callback_group=self.callback_group,
        )

        self.yolo_start_client = self.create_client(
            Trigger,
            self.yolo_config.get('start_service', '/yolo/start'),
            callback_group=self.callback_group,
        )
        self.yolo_stop_client = self.create_client(
            Trigger,
            self.yolo_config.get('stop_service', '/yolo/stop'),
            callback_group=self.callback_group,
        )

        self.navigator = BasicNavigator()
        self.inspection_records = []

        self.require_initial_pose = as_bool(
            self.task_config.get('require_initial_pose', True),
            True,
        )
        if self.require_initial_pose:
            self.set_state('WAIT_INITIAL_POSE', 'set initial pose in RViz before start')
        else:
            self.set_state('READY', 'waiting for start')

        self.get_logger().info(f'YOLO patrol config={self.config_path}')
        self.get_logger().info(f'Course config={self.course_config_path}')
        self.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active.')
        self.publish_markers()

    def load_course_config(self):
        course_config_file = self.config.get(
            'course_config_file',
            '/home/ubuntu/ros2_ws/src/course_design/config/course_design.yaml',
        )
        course_config, path = load_yaml(Path(course_config_file))
        return course_config, path

    def set_state(self, state, reason=''):
        self.state = state
        msg = f'STATE {state}' + (f': {reason}' if reason else '')
        self.get_logger().info(msg)
        self.status_pub.publish(String(data=msg))

    def publish_markers(self):
        publish_waypoint_markers(self, self.marker_pub, self.course_config)

    def initial_pose_callback(self, msg):
        self.initial_pose_received = True
        self.get_logger().info(
            'Initial pose received: '
            f'x={msg.pose.pose.position.x:.3f}, y={msg.pose.pose.position.y:.3f}'
        )
        if self.state == 'WAIT_INITIAL_POSE':
            self.set_state('READY', 'initial pose received')

    def object_callback(self, msg):
        now = time.monotonic()
        with self.inspect_lock:
            self.last_yolo_msg_time = now
            self.total_yolo_msg_count += 1
            objects = list(msg.objects)
            if self.inspect_active and self.active_window is not None:
                self.active_window.add_objects(objects, now)
            elif as_bool(self.det_config.get('enable_moving_watch', True), True):
                self.moving_watch.add_objects(objects, now)

    def start_callback(self, _request, response):
        success, message = self.start_async()
        response.success = success
        response.message = message
        return response

    def start_topic_callback(self, msg):
        if not msg.data:
            return
        success, message = self.start_async()
        if not success:
            self.get_logger().error(f'Start rejected: {message}')

    def reset_callback(self, _request, response):
        self.reset_event.set()
        response.success = True
        response.message = 'reset accepted'
        self.get_logger().info('Reset received from ~/reset')
        return response

    def reset_topic_callback(self, msg):
        if msg.data:
            self.reset_event.set()
            self.get_logger().info('Reset received from topic')

    def start_async(self):
        if self.require_initial_pose and not self.initial_pose_received:
            self.set_state('WAIT_INITIAL_POSE', 'start rejected: no /initialpose yet')
            return False, 'set initial pose in RViz first'
        if not self.task_lock.acquire(blocking=False):
            return False, 'YOLO patrol already running'
        self.running = True
        self.task_thread = threading.Thread(target=self.run_task_safely, daemon=True)
        self.task_thread.start()
        return True, 'YOLO patrol started'

    def run_task_safely(self):
        try:
            success, message = self.run_patrol()
            if success:
                self.set_state('DONE', message)
            else:
                self.set_state('ERROR', message)
        except Exception as exc:
            self.get_logger().exception(f'Unhandled YOLO patrol exception: {exc}')
            self.set_state('ERROR', str(exc))
        finally:
            publish_stop(self.stop_publishers)
            if as_bool(self.yolo_config.get('call_stop_service_after_done', False), False):
                self.call_yolo_service(self.yolo_stop_client, 'stop')
            if as_bool(self.report_config.get('print_summary_after_done', True), True):
                self.print_summary()
            self.running = False
            self.task_lock.release()

    def call_yolo_service(self, client, name):
        if not as_bool(self.yolo_config.get('call_start_service', True), True) and name == 'start':
            return True
        timeout_sec = float(self.yolo_config.get('service_timeout_sec', 3.0))
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(f'YOLO {name} service unavailable after {timeout_sec:.1f}s')
            return False
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            self.get_logger().warn(f'YOLO {name} service timeout')
            return False
        result = future.result()
        ok = bool(result and result.success)
        self.get_logger().info(f'YOLO {name} service result={ok}')
        return ok

    def run_patrol(self):
        self.set_state('RUNNING', 'YOLO enhanced patrol')
        self.call_yolo_service(self.yolo_start_client, 'start')

        patrol_points = self.task_config.get('patrol_points', ['pick_area', 'home'])
        patrol_count = int(self.task_config.get('patrol_count', 3))
        if not patrol_points:
            return False, 'patrol_points is empty'

        total_steps = patrol_count * len(patrol_points)
        step = 0
        for patrol_round in range(1, patrol_count + 1):
            for waypoint_name in patrol_points:
                step += 1
                self.get_logger().info(
                    f'YOLO_PATROL round={patrol_round}/{patrol_count} '
                    f'step={step}/{total_steps} target={waypoint_name}'
                )
                success, nav_result = self.go_named_point(waypoint_name)
                if not success:
                    return False, f'navigation failed target={waypoint_name}: {nav_result}'

                outcome = self.inspect_waypoint(patrol_round, step, waypoint_name)
                action = self.action_for_outcome(outcome)
                self.get_logger().info(
                    f'INSPECTION_DECISION target={waypoint_name} '
                    f'outcome={outcome["status"]} class={outcome.get("class", "")} '
                    f'action={action}'
                )
                if action in ('safety_stop', 'alert_stop'):
                    publish_stop(self.stop_publishers)
                    self.set_state('ALERT', f'{action}: {outcome}')
                    if as_bool(self.policy_config.get('wait_for_reset_on_stop', True), True):
                        ok = self.wait_for_reset()
                        if not ok:
                            return False, 'ROS shutdown while waiting for reset'
                    else:
                        return False, f'{action}: {outcome}'

        return True, f'YOLO patrol completed rounds={patrol_count}'

    def go_named_point(self, waypoint_name):
        try:
            pose = waypoint_to_pose(self, self.course_config, waypoint_name)
        except KeyError as exc:
            self.get_logger().error(str(exc))
            return False, str(exc)
        publish_current_goal_marker(self, self.marker_pub, self.course_config, waypoint_name)
        return run_navigation(
            self,
            self.navigator,
            pose,
            waypoint_name,
            self.timeout_sec,
            self.feedback_period_sec,
            self.stop_publishers,
        )

    def inspect_waypoint(self, patrol_round, step, waypoint_name):
        publish_stop(self.stop_publishers)
        settle_time = float(self.task_config.get('settle_time_sec', 1.0))
        if settle_time > 0:
            self.set_state('STOP_AND_OBSERVE', f'{waypoint_name}: settle {settle_time:.1f}s')
            time.sleep(settle_time)

        moving_summary = self.moving_watch.recent_summary(time.monotonic())
        summary = self.run_detection_window(float(self.det_config.get('observe_time_sec', 5.0)))

        retried = False
        if (
            summary.status in ('no_yolo_data', 'no_valid_object', 'unstable')
            and moving_summary
            and as_bool(self.det_config.get('retry_on_moving_candidate', True), True)
        ):
            retried = True
            self.get_logger().warn(
                f'Moving candidate existed before stop ({moving_summary}), '
                'but stop inspection did not confirm it. Retrying once.'
            )
            retry_summary = self.run_detection_window(
                float(self.det_config.get('retry_observe_time_sec', 3.0))
            )
            if retry_summary.confirmed or summary.status == 'no_yolo_data':
                summary = retry_summary

        outcome = self.make_outcome(summary, moving_summary, retried)
        outcome.update({
            'round': patrol_round,
            'step': step,
            'waypoint': waypoint_name,
        })
        self.inspection_records.append(outcome)
        self.status_pub.publish(String(data=f'INSPECTION {outcome}'))
        return outcome

    def run_detection_window(self, duration_sec):
        self.set_state('YOLO_DETECTING', f'observe {duration_sec:.1f}s')
        window = DetectionWindow(
            self.watch_classes,
            self.det_config.get('min_score', 0.6),
            self.det_config.get('stable_min_count', 3),
        )
        with self.inspect_lock:
            self.active_window = window
            self.inspect_active = True
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            time.sleep(0.1)
        with self.inspect_lock:
            self.inspect_active = False
            self.active_window = None
        return window.summarize()

    def make_outcome(self, summary, moving_summary, retried):
        if summary.confirmed:
            class_name = summary.confirmed_class
            risk = self.risk_for_class(class_name)
            return {
                'status': 'confirmed',
                'class': class_name,
                'risk': risk,
                'count': summary.count,
                'avg_score': round(summary.avg_score, 3),
                'max_score': round(summary.max_score, 3),
                'detail': summary.detail,
                'moving_candidate': moving_summary or '',
                'retried': retried,
            }

        if summary.status == 'no_yolo_data':
            status = 'no_yolo_data'
            detail = 'YOLO/camera produced no detection messages during stop observation'
        elif moving_summary:
            status = 'missed_after_moving'
            detail = (
                'moving watch saw candidate before arrival, but stop observation '
                f'was not confirmed: {summary.detail}'
            )
        elif summary.status == 'no_valid_object':
            status = 'no_target'
            detail = summary.detail
        else:
            status = summary.status
            detail = summary.detail
        return {
            'status': status,
            'class': '',
            'risk': 'none',
            'detail': detail,
            'moving_candidate': moving_summary or '',
            'retried': retried,
        }

    def risk_for_class(self, class_name):
        for risk, names in self.policy_config.items():
            if risk in ('no_detection_action', 'no_yolo_data_action',
                        'missed_after_moving_action', 'wait_for_reset_on_stop'):
                continue
            if class_name in (names or []):
                return risk
        return 'log_continue'

    def action_for_outcome(self, outcome):
        status = outcome.get('status')
        if status == 'confirmed':
            return outcome.get('risk', 'log_continue')
        if status == 'no_yolo_data':
            return self.policy_config.get('no_yolo_data_action', 'warn_continue')
        if status == 'missed_after_moving':
            return self.policy_config.get('missed_after_moving_action', 'warn_continue')
        if status in ('no_target', 'no_valid_object', 'unstable'):
            return self.policy_config.get('no_detection_action', 'continue')
        return 'warn_continue'

    def wait_for_reset(self):
        self.reset_event.clear()
        self.set_state('WAIT_RESET', 'publish /yolo_patrol/reset true or call ~/reset')
        while rclpy.ok():
            if self.reset_event.wait(timeout=0.2):
                self.set_state('RUNNING', 'reset received, continue patrol')
                return True
        return False

    def print_summary(self):
        self.get_logger().info('========== YOLO PATROL SUMMARY ==========')
        if not self.inspection_records:
            self.get_logger().info('No inspection records.')
        for record in self.inspection_records:
            self.get_logger().info(str(record))
        self.get_logger().info('=========================================')


def main(args=None):
    rclpy.init(args=args)
    node = YoloPatrolNode()
    executor = MultiThreadedExecutor(num_threads=3)
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
