#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

bridge = CvBridge()

rgb = None
depth = None
frame_id = 0

save_path = "dataset"
os.makedirs(save_path + "/rgb", exist_ok=True)
os.makedirs(save_path + "/depth", exist_ok=True)


def rgb_callback(msg):
    global rgb
    rgb = bridge.imgmsg_to_cv2(msg, "bgr8")


def depth_callback(msg):
    global depth
    depth = bridge.imgmsg_to_cv2(msg, "passthrough")


def save_loop():
    global frame_id
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if rgb is not None and depth is not None:

            rgb_file = f"{save_path}/rgb/{frame_id:06d}.png"
            depth_file = f"{save_path}/depth/{frame_id:06d}.png"

            cv2.imwrite(rgb_file, rgb)
            cv2.imwrite(depth_file, depth)

            print("Saved frame", frame_id)
            frame_id += 1

        rate.sleep()


rospy.init_node("rgbd_saver")

rospy.Subscriber("/camera/color/image_raw", Image, rgb_callback)
rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_callback)

save_loop()