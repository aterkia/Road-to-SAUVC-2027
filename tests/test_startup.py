"""Run against the installed workspace; verifies real processes and ROS discovery."""
import signal
import subprocess
import time
import unittest

import rclpy


class StartupTest(unittest.TestCase):
    def test_all_nodes_discovered(self):
        rclpy.init()
        node = rclpy.create_node('startup_test')
        process = subprocess.Popen(
            ['ros2', 'launch', 'bootstrap', 'auv.launch.py'], start_new_session=True)
        try:
            deadline = time.monotonic() + 20
            expected = {'helm', 'wayfinder', 'spectra', 'orchestrator'}
            while time.monotonic() < deadline:
                self.assertIsNone(process.poll(), 'launch exited prematurely')
                if expected <= set(node.get_node_names()):
                    break
                rclpy.spin_once(node, timeout_sec=0.1)
            self.assertTrue(expected <= set(node.get_node_names()))
        finally:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=10)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
