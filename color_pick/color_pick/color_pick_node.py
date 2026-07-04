#!/usr/bin/env python3
# encoding: utf-8
# @date:2026/07/03
# 自动识别色块并抓取抬起，检测掉落后恢复，成功后旋转180度丢弃

import os
import cv2
import time
import queue
import signal
import threading
import numpy as np
import rclpy

from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from xf_mic_asr_offline import voice_play
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition
from interfaces.msg import ColorsInfo, ColorDetect, ROI
from rclpy.callback_groups import ReentrantCallbackGroup
from interfaces.srv import SetColorDetectParam, SetCircleROI
from servo_controller.action_group_controller import ActionGroupController


class ColorPickNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(
            name,
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self.name = name
        self.running = True
        self.start = False
        self.start_pick = False
        self.in_motion = False
        self.debug = bool(self.get_param_value("debug", False))
        self.broadcast = bool(self.get_param_value("broadcast", False))
        self.language = os.environ.get("ASR_LANGUAGE", "en")
        self.camera_type = os.environ.get("DEPTH_CAMERA_TYPE", "")
        self.detect_type = str(self.get_param_value("detect_type", "rect"))
        self.enable_display = bool(self.get_param_value("enable_display", False))
        self.pick_confirm_frames = int(self.get_param_value("pick_confirm_frames", 25))
        self.drop_confirm_frames = int(self.get_param_value("drop_confirm_frames", 4))
        self.drop_check_timeout = float(self.get_param_value("drop_check_timeout", 1.2))
        self.detect_expire = float(self.get_param_value("detect_expire", 0.4))
        self.max_drop_retry = int(self.get_param_value("max_drop_retry", 3))
        self.min_detect_size = float(self.get_param_value("min_detect_size", 10.0))
        if not os.environ.get("DISPLAY"):
            self.enable_display = False
        self.pick_roi = self.load_pick_roi()
        self.target_colors = ("green", "blue")

        self.center = None
        self.color = ""
        self.target_color = ""
        self.last_detect_stamp = 0.0
        self.pick_stable_count = 0
        self.drop_retry = 0

        self.state_lock = threading.Lock()
        self.image_queue = queue.Queue(maxsize=2)
        self.display_queue = queue.Queue(maxsize=1)
        signal.signal(signal.SIGINT, self.shutdown)

        self.create_subscription(ColorsInfo, "/color_detect/color_info", self.get_color_callback, 1)
        self.create_subscription(Image, "/color_detect/image_result", self.image_callback, 1)

        timer_cb_group = ReentrantCallbackGroup()
        self.create_service(Trigger, "~/start", self.start_srv_callback, callback_group=timer_cb_group)
        self.create_service(Trigger, "~/stop", self.stop_srv_callback, callback_group=timer_cb_group)

        self.controller = ActionGroupController(
            self.create_publisher(ServosPosition, "servo_controller", 1),
            "/home/ubuntu/software/arm_pc/ActionGroups",
        )
        self.client = self.create_client(Trigger, "/controller_manager/init_finish")
        self.client.wait_for_service()
        self.run_action_safe("pick_init")

        self.set_color_client = self.create_client(
            SetColorDetectParam, "/color_detect/set_param", callback_group=timer_cb_group
        )
        self.set_roi_client = self.create_client(
            SetCircleROI, "/color_detect/set_circle_roi", callback_group=timer_cb_group
        )
        self.set_color_client.wait_for_service()
        self.set_roi_client.wait_for_service()

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def get_param_value(self, name, default):
        value = self.get_parameter(name).value
        return default if value is None else value

    def load_pick_roi(self):
        defaults = {"y_min": 240, "y_max": 340, "x_min": 260, "x_max": 380}
        pick_roi = self.get_parameters_by_prefix("roi")
        return [
            pick_roi["y_min"].value if "y_min" in pick_roi and pick_roi["y_min"].value is not None else defaults["y_min"],
            pick_roi["y_max"].value if "y_max" in pick_roi and pick_roi["y_max"].value is not None else defaults["y_max"],
            pick_roi["x_min"].value if "x_min" in pick_roi and pick_roi["x_min"].value is not None else defaults["x_min"],
            pick_roi["x_max"].value if "x_max" in pick_roi and pick_roi["x_max"].value is not None else defaults["x_max"],
        ]

    def init_process(self):
        self.timer.cancel()

        if self.debug:
            self.pick_roi = [200, 340, 240, 400]
            self.run_action_safe("pick_debug")
            time.sleep(3)
            self.run_action_safe("pick_init")
            time.sleep(1)

        if self.get_param_value("start", False):
            self.start_srv_callback(Trigger.Request(), Trigger.Response())

        threading.Thread(target=self.pick, daemon=True).start()
        threading.Thread(target=self.main, daemon=True).start()
        if self.enable_display:
            threading.Thread(target=self.display_loop, daemon=True).start()
        self.create_service(Trigger, "~/init_finish", self.get_node_state)
        self.get_logger().info("\033[1;32m%s\033[0m" % "start")

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum, frame):
        self.running = False

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()
            time.sleep(0.01)
        return None

    def start_srv_callback(self, request, response):
        self.get_logger().info("\033[1;32m%s\033[0m" % "start color pick")
        roi = ROI()
        roi.x_min = int(self.pick_roi[2] - 20)
        roi.x_max = int(self.pick_roi[3] + 20)
        roi.y_min = int(self.pick_roi[0] - 20)
        roi.y_max = int(self.pick_roi[1] + 20)
        msg = SetCircleROI.Request()
        msg.data = roi

        res = self.send_request(self.set_roi_client, msg)
        if res and res.success:
            self.get_logger().info("\033[1;32m%s\033[0m" % "set roi success")
        else:
            self.get_logger().warn("set roi failed")

        msg = SetColorDetectParam.Request()
        msg.data = []
        for color_name in self.target_colors:
            color_msg = ColorDetect()
            color_msg.color_name = color_name
            color_msg.detect_type = self.detect_type
            msg.data.append(color_msg)

        res = self.send_request(self.set_color_client, msg)
        if res and res.success:
            self.get_logger().info("\033[1;32m%s\033[0m" % "set color success")
        else:
            self.get_logger().warn("set color failed")

        self.reset_tracking()
        self.drop_retry = 0
        self.start = True

        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info("\033[1;32m%s\033[0m" % "stop color pick")
        self.start = False
        self.start_pick = False
        self.in_motion = False
        self.target_color = ""
        self.reset_tracking()

        res = self.send_request(self.set_color_client, SetColorDetectParam.Request())
        if res and res.success:
            self.get_logger().info("\033[1;32m%s\033[0m" % "clear color success")
        else:
            self.get_logger().warn("clear color failed")

        response.success = True
        response.message = "stop"
        return response

    def reset_tracking(self):
        with self.state_lock:
            self.center = None
            self.color = ""
            self.last_detect_stamp = 0.0
        self.pick_stable_count = 0

    def get_color_callback(self, msg):
        valid_targets = []
        for target in msg.data:
            color = getattr(target, "color", "")
            radius = float(getattr(target, "radius", 0.0))
            if color not in self.target_colors:
                continue
            if radius < self.min_detect_size and self.detect_type == "circle":
                continue
            valid_targets.append(target)

        if valid_targets:
            target = max(valid_targets, key=lambda item: float(getattr(item, "radius", 0.0)))
            with self.state_lock:
                self.center = target
                self.color = getattr(target, "color", "")
                self.last_detect_stamp = time.monotonic()
        else:
            with self.state_lock:
                self.center = None
                self.color = ""
                self.last_detect_stamp = time.monotonic()

    def detection_snapshot(self):
        with self.state_lock:
            return self.center, self.color, self.last_detect_stamp

    def in_pick_roi(self, x, y):
        return self.pick_roi[2] < x < self.pick_roi[3] and self.pick_roi[0] < y < self.pick_roi[1]

    def detection_in_roi(self, expected_color=None, after_stamp=0.0):
        center, color, stamp = self.detection_snapshot()
        if stamp < after_stamp or time.monotonic() - stamp > self.detect_expire:
            return False
        if center is None or color == "":
            return False
        if expected_color is not None and color != expected_color:
            return False
        return self.in_pick_roi(int(center.x), int(center.y))

    def pick(self):
        while self.running:
            if not self.start_pick:
                time.sleep(0.01)
                continue

            target_color = self.target_color
            self.start = False
            self.in_motion = True
            self.get_logger().info("\033[1;32mstart pick: %s\033[0m" % target_color)

            try:
                if not self.run_action_safe("pick"):
                    self.get_logger().error("pick action failed")
                    continue

                if self.broadcast:
                    try:
                        voice_play.play(target_color, language=self.language)
                    except Exception as exc:
                        self.get_logger().warn("voice broadcast failed: %s" % exc)

                pick_finish_stamp = time.monotonic()
                dropped = self.detect_drop_and_recover(target_color, pick_finish_stamp)
                if not dropped:
                    self.drop_retry = 0
                    self.discard_block()
                else:
                    self.drop_retry += 1
                    self.get_logger().warn(
                        "block dropped after pick, retry %d/%d" % (self.drop_retry, self.max_drop_retry)
                    )
                    if self.drop_retry >= self.max_drop_retry:
                        self.get_logger().error("drop retry limit reached, waiting for new stable target")
                        self.drop_retry = 0
            finally:
                self.start_pick = False
                self.in_motion = False
                self.target_color = ""
                self.pick_stable_count = 0
                self.run_action_safe("pick_init")
                time.sleep(0.3)
                if self.running:
                    self.start = True

    def detect_drop_and_recover(self, expected_color, after_stamp):
        deadline = time.monotonic() + self.drop_check_timeout
        hits = 0

        while self.running and time.monotonic() < deadline:
            if self.detection_in_roi(expected_color=expected_color, after_stamp=after_stamp):
                hits += 1
                if hits >= self.drop_confirm_frames:
                    return True
            else:
                hits = 0
            time.sleep(0.05)

        return False

    def discard_block(self):
        for action_name in (
            "pick_rotate_180_discard",
            "pick_discard_180",
            "rotate_180_discard",
            "discard_180",
            "throw_180",
            "place_back",
            "place_center",
        ):
            if self.run_action_safe(action_name, quiet=True):
                self.get_logger().info("discard action: %s" % action_name)
                return True

        self.get_logger().warn("no discard action matched, arm will return to pick_init")
        return False

    def run_action_safe(self, action_name, quiet=False):
        try:
            self.controller.run_action(action_name)
            return True
        except Exception as exc:
            if not quiet:
                self.get_logger().warn("run action %s failed: %s" % (action_name, exc))
            return False

    def draw_overlay(self, image):
        cv2.rectangle(
            image,
            (self.pick_roi[2] - 20, self.pick_roi[0] - 20),
            (self.pick_roi[3] + 20, self.pick_roi[1] + 20),
            (0, 255, 255),
            2,
        )

        center, color, stamp = self.detection_snapshot()
        if center is not None and time.monotonic() - stamp <= self.detect_expire:
            x = int(center.x)
            y = int(center.y)
            cv2.rectangle(image, (x - 25, y - 25), (x + 25, y + 25), (0, 0, 255), 2)
            cv2.putText(
                image,
                color,
                (x + 10, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

        state_text = "RUN" if self.start else "PAUSE"
        if self.start_pick or self.in_motion:
            state_text = "PICKING"
        cv2.putText(image, state_text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)

    def push_display_image(self, image):
        if not self.enable_display:
            return

        if self.display_queue.full():
            try:
                self.display_queue.get_nowait()
            except queue.Empty:
                pass

        try:
            self.display_queue.put_nowait(image.copy())
        except queue.Full:
            pass

    def display_loop(self):
        try:
            cv2.namedWindow("color_pick", cv2.WINDOW_NORMAL)
        except cv2.error as exc:
            self.enable_display = False
            self.get_logger().warn("display disabled: %s" % exc)
            return

        while self.running and self.enable_display:
            try:
                image = self.display_queue.get(block=True, timeout=0.2)
            except queue.Empty:
                try:
                    cv2.waitKey(1)
                except cv2.error as exc:
                    self.enable_display = False
                    self.get_logger().warn("display disabled: %s" % exc)
                continue

            try:
                self.draw_overlay(image)
                cv2.imshow("color_pick", image)
                key = cv2.waitKey(1)
                if key == ord("q") or key == 27:
                    self.running = False
            except cv2.error as exc:
                self.enable_display = False
                self.get_logger().warn("display disabled: %s" % exc)

        if self.enable_display:
            cv2.destroyAllWindows()

    def main(self):
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                continue

            if self.start and not self.start_pick and not self.in_motion:
                center, color, stamp = self.detection_snapshot()
                if (
                    center is not None
                    and color in self.target_colors
                    and time.monotonic() - stamp <= self.detect_expire
                    and self.in_pick_roi(int(center.x), int(center.y))
                ):
                    self.pick_stable_count += 1
                    if self.pick_stable_count >= self.pick_confirm_frames:
                        self.target_color = color
                        self.start_pick = True
                        self.pick_stable_count = 0
                else:
                    self.pick_stable_count = 0

            if image is not None:
                self.push_display_image(image)

        self.run_action_safe("init", quiet=True)
        rclpy.shutdown()

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(
            shape=(ros_image.height, ros_image.width, 3),
            dtype=np.uint8,
            buffer=ros_image.data,
        ).copy()

        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(rgb_image)


def main():
    node = ColorPickNode("color_pick")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()


if __name__ == "__main__":
    main()
