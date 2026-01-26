#!/usr/bin/env python3
"""
Real Rover Hardware Launch System
- Proper camera transforms for D435 point cloud
- Support for new IFWATER stereo camera
- Simplified for real hardware only
"""

import os
import signal
import sys
import subprocess
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QLabel,
    QHBoxLayout, QGroupBox, QLineEdit
)
from PyQt5.QtGui import QFont, QColor, QPainter
from PyQt5.QtCore import Qt, QTimer


class StatusLight(QLabel):
    """LED-style status indicator"""
    def __init__(self, color="red"):
        super().__init__()
        self.color = color
        self.setFixedSize(20, 20)

    def set_color(self, color):
        self.color = color
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        if self.color == "green":
            painter.setBrush(QColor(0, 220, 0))
        elif self.color == "yellow":
            painter.setBrush(QColor(255, 210, 0))
        else:
            painter.setBrush(QColor(200, 0, 0))
        painter.setPen(Qt.black)
        painter.drawEllipse(2, 2, 16, 16)


def run_in_terminal(command):
    """Launch command in new terminal window"""
    return subprocess.Popen(
        ["gnome-terminal", "--", "bash", "-c", f"{command}; exec bash"],
        preexec_fn=os.setsid
    )


class RealRoverLauncher(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Real Rover Control System")
        self.setMinimumWidth(750)

        self.processes = {}
        self.status_lights = {}

        # Hardware configuration
        self.motor_ports = {
            'front_right': '/dev/ttyUSB0',
            'front_left': '/dev/ttyUSB1',
            'back_right': '/dev/ttyUSB2',
            'back_left': '/dev/ttyUSB3',
            'actuator_1': '/dev/ttyUSB4',
            'actuator_2': '/dev/ttyUSB5',
            'camera_rotation': '/dev/ttyUSB7'
        }

        main_layout = QVBoxLayout()

        # HEADER
        header = QLabel("Rover Hardware Control")
        header.setFont(QFont("Arial", 18, QFont.Bold))
        header.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(header)

        # HARDWARE CONFIGURATION
        config_group = QGroupBox("Hardware Configuration")
        config_layout = QVBoxLayout()
        
        port_layout = QHBoxLayout()
        port_layout.addWidget(QLabel("Motor Ports:"))
        self.port_config_btn = QPushButton("Configure Ports")
        self.port_config_btn.clicked.connect(self.configure_ports)
        port_layout.addWidget(self.port_config_btn)
        config_layout.addLayout(port_layout)
        
        # Display current port configuration
        self.port_display = QLabel(self._format_port_config())
        self.port_display.setStyleSheet("color: #666; font-size: 10px;")
        config_layout.addWidget(self.port_display)
        
        config_group.setLayout(config_layout)
        main_layout.addWidget(config_group)

        # STOP ALL
        stop_all_btn = QPushButton("EMERGENCY STOP ALL")
        stop_all_btn.setFont(QFont("Arial", 16))
        stop_all_btn.setStyleSheet("background-color: #ff4444; color: white;")
        stop_all_btn.clicked.connect(self.stop_all_processes)
        main_layout.addWidget(stop_all_btn)

        # QUICK LAUNCH OPTIONS
        quick_group = QGroupBox("Quick Launch Options")
        quick_layout = QVBoxLayout()
        
        # Full system launch button
        full_launch_btn = QPushButton("Launch Complete System (All-in-One)")
        full_launch_btn.setFont(QFont("Arial", 12, QFont.Bold))
        full_launch_btn.setStyleSheet("background-color: #4CAF50; color: white; padding: 10px;")
        full_launch_btn.clicked.connect(self.launch_complete_system)
        quick_layout.addWidget(full_launch_btn)
        
        # Camera test button
        camera_test_btn = QPushButton("Launch Camera Test (No Motors)")
        camera_test_btn.setFont(QFont("Arial", 12))
        camera_test_btn.setStyleSheet("background-color: #2196F3; color: white; padding: 10px;")
        camera_test_btn.clicked.connect(self.launch_camera_test)
        quick_layout.addWidget(camera_test_btn)
        
        quick_group.setLayout(quick_layout)
        main_layout.addWidget(quick_group)

        # INDIVIDUAL COMPONENTS
        components_group = QGroupBox("Individual Component Control")
        components_layout = QVBoxLayout()

        # Robot State Publisher with camera transforms (COMBINED)
        components_layout.addWidget(self.create_control_block(
            "transforms",
            "TF Transforms (base_link + camera frames)",
            """
cd ~/lunar_rover_ws
source install/setup.bash

# Start robot state publisher
ros2 run robot_state_publisher robot_state_publisher \\
    --ros-args \\
    -p robot_description:="<?xml version='1.0'?>
<robot name='lunar_rover'>
  <link name='base_link'>
    <visual><geometry><box size='0.5 0.3 0.2'/></geometry></visual>
  </link>
  <link name='camera_link'/>
  <joint name='base_to_camera' type='fixed'>
    <parent link='base_link'/><child link='camera_link'/>
    <origin xyz='0.15 0 0.2' rpy='0 0 0'/>
  </joint>
  <link name='camera_rear_link'/>
  <joint name='base_to_camera_rear' type='fixed'>
    <parent link='base_link'/><child link='camera_rear_link'/>
    <origin xyz='-0.15 0 0.2' rpy='0 0 3.14159265359'/>
  </joint>
</robot>" \\
    -p use_sim_time:=false &

RSP_PID=$!

# Wait for robot_state_publisher to start
sleep 1

# Camera optical frame transforms (CRITICAL for point cloud)
ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5707963267948966 0 -1.5707963267948966 camera_link camera_depth_optical_frame &
ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5707963267948966 0 -1.5707963267948966 camera_link camera_color_optical_frame &
ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5707963267948966 0 -1.5707963267948966 camera_rear_link camera_rear_depth_optical_frame &

# Wait for all processes
wait $RSP_PID
"""
        ))

        # Motor controller
        components_layout.addWidget(self.create_control_block(
            "motor_controller",
            "Motor Controller (4-Wheel Drive + 2 Actuators)",
            f"""
cd ~/lunar_rover_ws
source install/setup.bash
ros2 run lunar_robot_hardware motor_controller_node \\
    --ros-args \\
    -p fr_port:={self.motor_ports['front_right']} \\
    -p fl_port:={self.motor_ports['front_left']} \\
    -p br_port:={self.motor_ports['back_right']} \\
    -p bl_port:={self.motor_ports['back_left']} \\
    -p use_sim_time:=false
"""
        ))

        # Front camera (D435)
        components_layout.addWidget(self.create_control_block(
            "front_camera",
            "Front Camera (D435 - RGB + Depth + Point Cloud)",
            """
cd ~/lunar_rover_ws
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py \\
    camera_name:=camera \\
    camera_namespace:=camera \\
    enable_depth:=true \\
    enable_color:=true \\
    pointcloud.enable:=true \\
    align_depth.enable:=true \\
    depth_module.profile:=640x480x30 \\
    rgb_camera.profile:=640x480x30
"""
        ))

        # Rear camera (IFWATER Stereo)
        components_layout.addWidget(self.create_control_block(
            "rear_camera",
            "Rear Camera (IFWATER Stereo - Depth + RGB)",
            """
cd ~/lunar_rover_ws
source install/setup.bash
# IFWATER camera launch - adjust device path as needed
ros2 run usb_cam usb_cam_node_exe \\
    --ros-args \\
    -p video_device:=/dev/video2 \\
    -r __ns:=/camera_rear \\
    -r __node:=camera_rear \\
    -p image_width:=1280 \\
    -p image_height:=720 \\
    -p framerate:=30.0 \\
    -p camera_frame_id:=camera_rear_color_optical_frame
"""
        ))

        # Navigation (optional)
        components_layout.addWidget(self.create_control_block(
            "navigation",
            "Autonomous Navigator (Point-Click + Obstacles)",
            """
cd ~/lunar_rover_ws
source install/setup.bash
ros2 run lunar_robot_autonomous unified_navigator \\
    --ros-args -p use_sim_time:=false
"""
        ))

        # RViz
        components_layout.addWidget(self.create_control_block(
            "rviz",
            "RViz Visualization",
            """
cd ~/lunar_rover_ws
source install/setup.bash
ros2 run rviz2 rviz2 -d ~/lunar_rover_ws/hardware_navigation.rviz \\
    --ros-args -p use_sim_time:=false
"""
        ))

        # Teleop with actuators
        components_layout.addWidget(self.create_control_block(
            "teleop",
            "Keyboard Teleop (Drive + Camera + Actuators)",
            """
cd ~/lunar_rover_ws
python3 teleop_keyboard.py
"""
        ))

        components_group.setLayout(components_layout)
        main_layout.addWidget(components_group)

        self.setLayout(main_layout)

        # Timer for status updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_process_status)
        self.timer.start(500)

    def launch_complete_system(self):
        """Launch the complete rover system"""
        # Start transforms first
        self.start_process("transforms", self.get_transforms_command())
        
        # Wait a bit for transforms
        QTimer.singleShot(2000, lambda: self.start_process("motor_controller", self.get_motor_command()))
        QTimer.singleShot(3000, lambda: self.start_process("front_camera", self.get_front_camera_command()))
        QTimer.singleShot(4000, lambda: self.start_process("rear_camera", self.get_rear_camera_command()))
        QTimer.singleShot(5000, lambda: self.start_process("rviz", self.get_rviz_command()))

    def launch_camera_test(self):
        """Launch camera test mode"""
        self.start_process("transforms", self.get_transforms_command())
        QTimer.singleShot(2000, lambda: self.start_process("front_camera", self.get_front_camera_command()))
        QTimer.singleShot(3000, lambda: self.start_process("rviz", self.get_rviz_command()))

    def get_transforms_command(self):
        """Get the transforms command"""
        return """
cd ~/lunar_rover_ws
source install/setup.bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="<?xml version='1.0'?><robot name='lunar_rover'><link name='base_link'><visual><geometry><box size='0.5 0.3 0.2'/></geometry></visual></link><link name='camera_link'/><joint name='base_to_camera' type='fixed'><parent link='base_link'/><child link='camera_link'/><origin xyz='0.15 0 0.2' rpy='0 0 0'/></joint><link name='camera_rear_link'/><joint name='base_to_camera_rear' type='fixed'><parent link='base_link'/><child link='camera_rear_link'/><origin xyz='-0.15 0 0.2' rpy='0 0 3.14159265359'/></joint></robot>" -p use_sim_time:=false &
sleep 1
ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5707963267948966 0 -1.5707963267948966 camera_link camera_depth_optical_frame &
ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5707963267948966 0 -1.5707963267948966 camera_link camera_color_optical_frame &
ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5707963267948966 0 -1.5707963267948966 camera_rear_link camera_rear_depth_optical_frame &
wait
"""

    def get_motor_command(self):
        return f"""
cd ~/lunar_rover_ws
source install/setup.bash
ros2 run lunar_robot_hardware motor_controller_node --ros-args -p fr_port:={self.motor_ports['front_right']} -p fl_port:={self.motor_ports['front_left']} -p br_port:={self.motor_ports['back_right']} -p bl_port:={self.motor_ports['back_left']} -p use_sim_time:=false
"""

    def get_front_camera_command(self):
        return """
cd ~/lunar_rover_ws
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py camera_name:=camera camera_namespace:=camera enable_depth:=true enable_color:=true pointcloud.enable:=true align_depth.enable:=true
"""

    def get_rear_camera_command(self):
        return """
cd ~/lunar_rover_ws
source install/setup.bash
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2 -r __ns:=/camera_rear -r __node:=camera_rear -p image_width:=1280 -p image_height:=720 -p framerate:=30.0 -p camera_frame_id:=camera_rear_depth_optical_frame
"""

    def get_rviz_command(self):
        return """
cd ~/lunar_rover_ws
source install/setup.bash
ros2 run rviz2 rviz2 -d ~/lunar_rover_ws/hardware_navigation.rviz --ros-args -p use_sim_time:=false
"""

    def _format_port_config(self):
        """Format port configuration for display"""
        return (f"FR: {self.motor_ports['front_right']} | FL: {self.motor_ports['front_left']} | "
                f"BR: {self.motor_ports['back_right']} | BL: {self.motor_ports['back_left']} | "
                f"ACT1: {self.motor_ports['actuator_1']} | ACT2: {self.motor_ports['actuator_2']} | "
                f"CAM: {self.motor_ports['camera_rotation']}")

    def configure_ports(self):
        """Open port configuration dialog"""
        from PyQt5.QtWidgets import QDialog, QFormLayout, QDialogButtonBox
        
        dialog = QDialog(self)
        dialog.setWindowTitle("Configure Motor Ports")
        layout = QFormLayout()
        
        port_inputs = {}
        for motor, port in self.motor_ports.items():
            input_field = QLineEdit(port)
            layout.addRow(f"{motor.replace('_', ' ').title()}:", input_field)
            port_inputs[motor] = input_field
        
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addRow(buttons)
        
        dialog.setLayout(layout)
        
        if dialog.exec_():
            for motor, input_field in port_inputs.items():
                self.motor_ports[motor] = input_field.text()
            self.port_display.setText(self._format_port_config())

    def create_control_block(self, name, label_text, command):
        """Create control group with start/stop buttons"""
        group = QGroupBox(label_text)
        layout = QHBoxLayout()

        light = StatusLight("red")
        self.status_lights[name] = light
        layout.addWidget(light)

        btn_start = QPushButton("Start")
        btn_start.clicked.connect(lambda _, n=name, c=command: self.start_process(n, c))
        layout.addWidget(btn_start)

        btn_stop = QPushButton("Stop")
        btn_stop.clicked.connect(lambda _, n=name: self.stop_process(n))
        layout.addWidget(btn_stop)

        group.setLayout(layout)
        return group

    def start_process(self, name, command):
        """Start a process"""
        if name in self.processes and self.processes[name] is not None:
            self.stop_process(name)

        self.status_lights[name].set_color("yellow")
        proc = run_in_terminal(command)
        self.processes[name] = proc

    def stop_process(self, name):
        """Stop a process"""
        proc = self.processes.get(name)
        if proc is not None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception:
                pass
        self.processes[name] = None
        self.status_lights[name].set_color("red")

    def stop_all_processes(self):
        """Emergency stop all processes"""
        for name in list(self.processes.keys()):
            self.stop_process(name)

    def update_process_status(self):
        """Update status lights based on process state"""
        for name, proc in self.processes.items():
            if proc is None:
                self.status_lights[name].set_color("red")
            else:
                if proc.poll() is None:
                    self.status_lights[name].set_color("green")
                else:
                    self.status_lights[name].set_color("red")


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RealRoverLauncher()
    window.show()
    sys.exit(app.exec_())