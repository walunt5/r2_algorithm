"""Integration tests for the light-signal wait action server."""

from pathlib import Path
import os
import sys
import threading
import time

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool

from r2_light_interfaces.action import WaitForLightSignal


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from light_signal_wait_action_server import LightSignalWaitActionServer  # noqa: E402


def _wait_for_future(future, timeout_sec=2.0):
    deadline = time.monotonic() + timeout_sec
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert future.done(), "ROS future did not complete before the test timeout"
    return future.result()


@pytest.fixture(scope="module")
def action_fixture():
    log_dir = Path("/tmp/r2_light_signal_test_logs")
    log_dir.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_LOG_DIR"] = str(log_dir)
    rclpy.init()
    server = LightSignalWaitActionServer()
    client_node = Node("test_light_signal_wait_action_client")
    publisher = client_node.create_publisher(Bool, "/light_signal/on", 10)
    client = ActionClient(
        client_node,
        WaitForLightSignal,
        "/r2_light_signal/wait",
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    executor.add_node(client_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    assert client.wait_for_server(timeout_sec=2.0)

    yield client, publisher

    executor.shutdown()
    spin_thread.join(timeout=2.0)
    client_node.destroy_node()
    server.destroy_node()
    rclpy.shutdown()


def _send_goal(client, timeout_sec, feedback_callback=None):
    goal = WaitForLightSignal.Goal()
    goal.timeout_sec = timeout_sec
    future = client.send_goal_async(goal, feedback_callback=feedback_callback)
    return _wait_for_future(future)


def test_true_published_before_goal_is_ignored(action_fixture) -> None:
    client, publisher = action_fixture
    publisher.publish(Bool(data=True))
    time.sleep(0.15)

    goal_handle = _send_goal(client, 0.2)
    assert goal_handle.accepted
    wrapped = _wait_for_future(goal_handle.get_result_async())

    assert wrapped.status == GoalStatus.STATUS_ABORTED
    assert not wrapped.result.success
    assert "no new light signal" in wrapped.result.message


def test_new_false_feedback_keeps_goal_running_until_timeout(action_fixture) -> None:
    client, publisher = action_fixture
    feedback = []
    goal_handle = _send_goal(
        client,
        0.3,
        feedback_callback=lambda msg: feedback.append(msg.feedback),
    )
    assert goal_handle.accepted
    for _ in range(3):
        publisher.publish(Bool(data=False))
        time.sleep(0.05)

    wrapped = _wait_for_future(goal_handle.get_result_async())

    assert wrapped.status == GoalStatus.STATUS_ABORTED
    assert any(item.signal_received and not item.detected for item in feedback)
    assert "none was true" in wrapped.result.message


def test_new_true_succeeds(action_fixture) -> None:
    client, publisher = action_fixture
    goal_handle = _send_goal(client, 1.0)
    assert goal_handle.accepted
    publisher.publish(Bool(data=False))
    time.sleep(0.05)
    publisher.publish(Bool(data=True))

    wrapped = _wait_for_future(goal_handle.get_result_async())

    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped.result.success


def test_cancel_is_honored(action_fixture) -> None:
    client, _ = action_fixture
    goal_handle = _send_goal(client, 1.0)
    assert goal_handle.accepted
    cancel_response = _wait_for_future(goal_handle.cancel_goal_async())
    assert cancel_response.goals_canceling

    wrapped = _wait_for_future(goal_handle.get_result_async())
    assert wrapped.status == GoalStatus.STATUS_CANCELED
    assert not wrapped.result.success


def test_concurrent_goal_is_rejected(action_fixture) -> None:
    client, _ = action_fixture
    first_goal = _send_goal(client, 1.0)
    assert first_goal.accepted

    second_goal = _send_goal(client, 0.2)
    assert not second_goal.accepted

    _wait_for_future(first_goal.cancel_goal_async())
    wrapped = _wait_for_future(first_goal.get_result_async())
    assert wrapped.status == GoalStatus.STATUS_CANCELED
