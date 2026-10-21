import cv2
import numpy as np
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool
from vision_msgs.msg import Detection2DArray

from ros_test import RosTest


def gate_image(center=320, width=200):
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    for x in (center - width // 2, center + width // 2):
        cv2.rectangle(image, (x - 8, 100), (x + 8, 380), (0, 128, 255), -1)
    return image


class VisionTest(RosTest):
    def test_finals_detection_labels_and_headers(self):
        self.start('spectra', ('enable_finals', 'true'))
        detections = self.observe(
            '/perception/detections', Detection2DArray, qos_profile_sensor_data
        )
        camera = self.node.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.circle(image, (120, 240), 50, (255, 0, 0), -1)
        for x, color in [(350, (0, 0, 255)), (450, (0, 255, 0)), (550, (0, 255, 255))]:
            cv2.rectangle(image, (x, 100), (x + 20, 380), color, -1)
        bridge = CvBridge()

        def publish():
            msg = bridge.cv2_to_imgmsg(image, encoding='bgr8')
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_optical_frame'
            camera.publish(msg)

        timer = self.node.create_timer(0.05, publish)
        self.wait(lambda: detections)
        labels = {d.results[0].hypothesis.class_id for d in detections[-1].detections}
        self.assertEqual(labels, {'drum', 'flare_red', 'flare_green', 'flare_yellow'})
        self.assertEqual(detections[-1].header.frame_id, 'camera_optical_frame')
        self.assertTrue(all(d.header == detections[-1].header for d in detections[-1].detections))
        self.node.destroy_timer(timer)

    def test_image_to_guidance_and_camera_loss(self):
        self.start('helm')
        self.start('wayfinder')
        self.start('spectra')
        detections = self.observe(
            '/perception/detections', Detection2DArray, qos_profile_sensor_data
        )
        status = self.observe('/navigation/status', String)
        output = self.observe('/vehicle/applied_cmd', Twist)
        health = self.observe('/vehicle/health', DiagnosticArray)
        camera = self.node.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        frame = [np.zeros((480, 640, 3), dtype=np.uint8)]
        bridge = CvBridge()

        def publish():
            msg = bridge.cv2_to_imgmsg(frame[0], encoding='bgr8')
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_optical_frame'
            camera.publish(msg)

        timer = self.node.create_timer(0.05, publish)
        self.wait(lambda: detections and health)
        self.assertTrue(self.call('/vehicle/arm', SetBool, SetBool.Request(data=True)).success)
        self.wait_armed_delivery(health)
        self.assertTrue(self.goal('PASS_GATE').accepted)
        self.wait(lambda: output and output[-1].angular.z > 0)
        frame[0] = gate_image(center=450)
        self.wait(lambda: any('ALIGN' in m.data for m in status))
        self.wait(lambda: output[-1].angular.z < 0)
        self.assertEqual(output[-1].linear.x, 0)
        frame[0] = gate_image()
        self.wait(lambda: any('APPROACH' in m.data for m in status))
        self.wait(lambda: output[-1].linear.x > 0)
        self.assertEqual(detections[-1].header.frame_id, 'camera_optical_frame')
        self.assertEqual(detections[-1].detections[0].results[0].hypothesis.class_id, 'gate')
        self.node.destroy_timer(timer)
        self.wait(lambda: any('FAILED: camera stale' in m.data for m in status), timeout=2)
        self.wait(lambda: output[-1].linear.x == 0)
        self.assertFalse(any('COMPLETED' in m.data for m in status))

    def test_stale_malformed_and_wrong_frame_images_not_published(self):
        self.start('spectra')
        detections = self.observe(
            '/perception/detections', Detection2DArray, qos_profile_sensor_data
        )
        camera = self.node.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.wait(lambda: camera.get_subscription_count())
        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(gate_image(), encoding='bgr8')
        msg.header.frame_id = 'camera_optical_frame'
        camera.publish(msg)  # zero/old timestamp
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'wrong_frame'
        camera.publish(msg)
        msg.header.frame_id = 'camera_optical_frame'
        msg.data = []
        camera.publish(msg)
        # The valid blank frame is an observable processing barrier.
        valid = bridge.cv2_to_imgmsg(np.zeros((480, 640, 3), dtype=np.uint8), encoding='bgr8')
        valid.header.frame_id = 'camera_optical_frame'
        valid.header.stamp = self.node.get_clock().now().to_msg()
        timer = self.node.create_timer(0.05, lambda: camera.publish(valid))
        self.wait(lambda: detections)
        self.node.destroy_timer(timer)
        self.assertTrue(all(not d.detections for d in detections))
