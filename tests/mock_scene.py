"""A fixed gate projected from mock ground truth, independent of navigation phases."""
import math

import cv2
import numpy as np
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class GateScene:
    def __init__(self, node):
        self.node = node
        self.pose = None
        self.enabled = True
        self.bridge = CvBridge()
        self.publisher = node.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.subscription = node.create_subscription(Odometry, '/vehicle/odometry', self.on_pose, qos_profile_sensor_data)
        self.timer = node.create_timer(0.05, self.publish)
        self.frames = 0

    def on_pose(self, odom):
        self.pose = odom.pose.pose

    def publish(self):
        if self.pose is None or not self.enabled:
            return
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        q = self.pose.orientation
        yaw = 2 * math.atan2(q.z, q.w)
        posts = []
        for world_y in (-0.25, 0.25):
            dx, dy = 1.0-self.pose.position.x, world_y-self.pose.position.y
            forward = dx*math.cos(yaw) + dy*math.sin(yaw)
            left = -dx*math.sin(yaw) + dy*math.cos(yaw)
            if forward <= 0.1:
                continue
            x = 320 - 320*left/forward
            h = min(400, 320*0.6/forward)
            width = max(5, min(30, int(320*0.025/forward)))
            if width < x < 640-width:
                posts.append((int(x), int(h), width))
        if len(posts) == 2:
            for x, h, w in posts:
                cv2.rectangle(image, (x-w, 240-h//2), (x+w, 240+h//2), (0, 128, 255), -1)
        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_optical_frame'
        self.publisher.publish(msg)
        self.frames += 1
