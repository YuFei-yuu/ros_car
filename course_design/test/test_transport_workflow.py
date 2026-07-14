from threading import Event
from types import SimpleNamespace
from unittest.mock import Mock

import course_design.navigation_utils as navigation_utils
from course_design.transport_workflow_node import (
    TransportWorkflowNode,
    parse_color_sequence,
    parse_positive_float,
)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_parse_color_sequence_supports_order_and_legacy_fallback():
    sequence, reason = parse_color_sequence({
        'color_sequence': ['GREEN', 'red', 'blue'],
    })
    assert reason == ''
    assert sequence == ['green', 'red', 'blue']

    sequence, reason = parse_color_sequence({'target_color': 'BLUE'})
    assert reason == ''
    assert sequence == ['blue']


def test_parse_color_sequence_rejects_empty_or_invalid_values():
    sequence, reason = parse_color_sequence({'color_sequence': []})
    assert sequence is None
    assert 'non-empty list' in reason

    sequence, reason = parse_color_sequence({'color_sequence': ['red', '']})
    assert sequence is None
    assert 'empty color' in reason


def test_parse_positive_float_rejects_non_positive_values():
    value, reason = parse_positive_float(
        {'backoff_speed_mps': 0}, 'backoff_speed_mps', 0.05)
    assert value is None
    assert 'positive number' in reason

    value, reason = parse_positive_float({}, 'backoff_duration_sec', 3.0)
    assert reason == ''
    assert value == 3.0


def test_run_color_cycle_preserves_configured_order():
    node = TransportWorkflowNode.__new__(TransportWorkflowNode)
    node.color_sequence = ['green', 'red', 'blue']
    node.goal_by_color = {
        'green': 'goal_green',
        'red': 'goal_red',
        'blue': 'goal_blue',
    }
    node.color = ''
    node.goal_name = ''
    node.set_color_client = object()
    node.pick_client = object()
    node.place_client = object()
    node.stop_publishers = [FakePublisher()]
    node.set_state = Mock()
    node.navigate = Mock(return_value=(True, 'navigation ok'))
    node.call_service = Mock(return_value=(True, 'service ok'))
    node.backoff = Mock(return_value=(True, 'backoff ok'))

    for index, color in enumerate(node.color_sequence, start=1):
        success, reason = node.run_color_cycle(index, color)
        assert success is True
        assert reason == 'service ok'

    navigation_calls = node.navigate.call_args_list
    assert [call.args[1] for call in navigation_calls] == [
        'pick_area', 'goal_green',
        'pick_area', 'goal_red',
        'pick_area', 'goal_blue',
    ]
    labels = [call.args[2] for call in node.call_service.call_args_list]
    assert labels == [
        'set target color', 'pick target', 'place target',
        'set target color', 'pick target', 'place target',
        'set target color', 'pick target', 'place target',
    ]


def test_run_timed_backoff_publishes_reverse_velocity_and_stop(monkeypatch):
    clock = SimpleNamespace(now=0.0)

    def monotonic():
        return clock.now

    def sleep(duration):
        clock.now += duration

    monkeypatch.setattr(navigation_utils.time, 'monotonic', monotonic)
    monkeypatch.setattr(navigation_utils.time, 'sleep', sleep)
    monkeypatch.setattr(navigation_utils.rclpy, 'ok', lambda: True)

    publisher = FakePublisher()
    node = SimpleNamespace(logger=SimpleNamespace(info=lambda _message: None))
    node.get_logger = lambda: node.logger
    success, reason = navigation_utils.run_timed_backoff(
        node, [publisher], 0.05, 0.15, 0.05, Event())

    assert success is True
    assert reason == 'completed'
    assert any(message.linear.x == -0.05 for message in publisher.messages)
    assert publisher.messages[-1].linear.x == 0.0


def test_run_timed_backoff_stops_when_cancelled(monkeypatch):
    monkeypatch.setattr(navigation_utils.rclpy, 'ok', lambda: True)
    publisher = FakePublisher()
    cancel_event = Event()
    cancel_event.set()
    node = SimpleNamespace(logger=SimpleNamespace(info=lambda _message: None))
    node.get_logger = lambda: node.logger

    success, reason = navigation_utils.run_timed_backoff(
        node, [publisher], 0.05, 3.0, 0.05, cancel_event)

    assert success is False
    assert reason == 'cancelled during backoff'
    assert publisher.messages[-1].linear.x == 0.0
