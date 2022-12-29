#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from toroboeye_camera.toroboeye_camera import Client


def main(args=None):

    # Create node
    rclpy.init(args=args)
    node = rclpy.create_node('toroboeye_example_node')

    # Create ToroboEye Client
    tc = Client(node)
    tc.wait_for_service()

    # Connect
    tc.connect('toroboeye-controller.local')

    # Get intrinsics
    res = tc.get_intrinsics()

    # Set parameters
    tc.set()

    # Get parameters
    tc.get()

    # Activate camera
    tc.activate()
    tc.wait_for_active()

    # Capture images
    tc.capture()
    frame = tc.wait_for_frame()

    # Save images
    bridge = CvBridge()
    color_image = bridge.imgmsg_to_cv2(frame.color_image, 'rgb8')
    depth_image = bridge.imgmsg_to_cv2(frame.depth_image, '16UC1')
    timestamp = frame.timestamp
    cv2.imwrite('color_image.jpg', color_image)
    cv2.imwrite('depth_image.png', depth_image)

    # Deactivate camera
    tc.deactivate()
    tc.wait_for_inactive()

    # Disconnect
    tc.disconnect()

    # Shutdown ROS2
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
