#!/bin/bash
echo "Stopping rear camera processes..."
pkill -f stereo_camera_publisher
pkill -f stereo_combiner
pkill -f "optimized_image_pipeline.*rear\|rear.*optimized_image_pipeline"
# Kill the pipeline instance subscribed to rear topic
for pid in $(pgrep -f optimized_image_pipeline); do
    grep -q "camera_rear" /proc/$pid/cmdline 2>/dev/null && kill $pid
done
sleep 2

source /opt/ros/jazzy/setup.bash
source ~/lunar_rover_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

echo "Starting stereo publisher..."
python3 ~/lunar_rover_ws/stereo_camera_publisher.py \
    --ros-args -p device:=/dev/video_stereo \
    -p width:=1600 -p height:=600 -p publish_rate:=10.0 &
sleep 3

echo "Starting combiner..."
python3 ~/lunar_rover_ws/stereo_combiner.py \
    --ros-args \
    -p left_crop_start:=0 -p left_crop_width:=800 \
    -p right_crop_start:=800 -p right_crop_width:=800 \
    -p publish_compressed:=true &
sleep 2

echo "Starting pipeline..."
python3 ~/lunar_rover_ws/optimized_image_pipeline.py \
    --ros-args \
    -p input_topic:=/camera_rear/stereo_combined/compressed \
    -p output_topic:=/camera_rear/stream/compressed \
    -p input_is_compressed:=true \
    -p jpeg_quality:=40 \
    -p decimation:=1 \
    -p buffer_delay_sec:=0.0 \
    -p target_fps:=10.0 &
sleep 3

echo "Verifying..."
ros2 topic hz /camera_rear/stream/compressed --spin-time 4
