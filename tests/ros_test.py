import signal
import subprocess
import time
import unittest

import rclpy
from rclpy.action import ActionClient
from aterkia_interfaces.action import NavigateTask
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter as RosParameter


class RosTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('integration_test')
        self.processes = []
        self.subscriptions = []
        self.actions = []
        self.addCleanup(self.cleanup)

    def cleanup(self):
        for process in reversed(self.processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        for client in self.actions:
            client.destroy()
        self.node.destroy_node()
        rclpy.shutdown()

    def start(self, package, *parameters):
        command = ['ros2', 'run', package, f'{package}_node', '--ros-args']
        for name, value in parameters:
            command += ['-p', f'{name}:={value}']
        self.processes.append(subprocess.Popen(command))

    def wait(self, predicate, timeout=5, message='condition not reached'):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return
            for process in self.processes:
                self.assertIsNone(process.poll(), 'ROS node exited unexpectedly')
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertTrue(predicate(), message)

    def observe(self, topic, msg_type, qos=10):
        messages = []
        self.subscriptions.append(self.node.create_subscription(msg_type, topic, messages.append, qos))
        return messages

    def call(self, service, srv_type, request):
        client = self.node.create_client(srv_type, service)
        self.wait(client.service_is_ready, message=service)
        future = client.call_async(request)
        self.wait(future.done)
        result = future.result()
        self.node.destroy_client(client)
        return result

    def parameter(self, node, name, value):
        response = self.call(f'/{node}/set_parameters', SetParameters,
                             SetParameters.Request(parameters=[RosParameter(name, value=value).to_parameter_msg()]))
        self.assertTrue(response.results[0].successful, response.results[0].reason)

    def goal(self, task, timeout=15.0, turn_radians=3.141592653589793, feedback=None):
        client = ActionClient(self.node, NavigateTask, '/navigation/task')
        self.actions.append(client)
        self.wait(client.server_is_ready, message='action server unavailable')
        request = NavigateTask.Goal(task=task, timeout=timeout, turn_radians=turn_radians)
        future = client.send_goal_async(request, feedback_callback=feedback)
        self.wait(future.done)
        return future.result()
