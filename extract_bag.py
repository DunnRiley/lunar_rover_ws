import pyrealsense2 as rs
import numpy as np
import cv2
import os

bag = "output.bag"

save_path = "dataset"
os.makedirs(save_path+"/rgb", exist_ok=True)
os.makedirs(save_path+"/depth", exist_ok=True)

pipeline = rs.pipeline()
config = rs.config()
config.enable_device_from_file(bag)

pipeline.start(config)

frame_id = 0

try:
    while True:

        frames = pipeline.wait_for_frames()

        depth = frames.get_depth_frame()
        color = frames.get_color_frame()

        if not depth or not color:
            continue

        depth_img = np.asanyarray(depth.get_data())
        color_img = np.asanyarray(color.get_data())

        cv2.imwrite(f"{save_path}/rgb/{frame_id:06d}.png", color_img)
        cv2.imwrite(f"{save_path}/depth/{frame_id:06d}.png", depth_img)

        frame_id += 1

except RuntimeError:
    pass

pipeline.stop()