#!/usr/bin/env python3
"""
Real Rover Hardware Launch System
Updated with proper transform support for RViz
"""

import os
import signal
import sys
import subprocess
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QLabel,
    QHBoxLayout, QGroupBox, QComboBox, QLineEdit
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
            'camera_rotation': '/dev/ttyUSB7'
        }

        main_layout = QVBoxLayout()

        # ==================== HEADER ====================
        header = QLabel("🤖 Real Rover Hardware Control")
        header.setFont(QFont("Arial", 18, QFont.Bold))
        header.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(header)

        # ==================== HARDWARE CONFIGURATION ====================
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

        # ==================== STOP ALL ====================
        stop_all_btn = QPushButton("🛑 EMERGENCY STOP ALL")
        stop_all_btn.setFont(QFont("Arial", 16))
        stop_all_btn.setStyleSheet("background-color: #ff4444; color: white;")
        stop_all_btn.clicked.connect(self.stop_all_processes)
        main_layout.addWidget(stop_all_btn)

        # ==================== QUICK LAUNCH OPTIONS ====================
        quick_group = QGroupBox("🚀 Quick Launch Options")
        quick_layout = QVBoxLayout()
        
        # Full system launch button
        full_launch_btn = QPushButton("▶ Launch Complete System (All-in-One)")
        full_launch_btn.setFont(QFont("Arial", 12, QFont.Bold))
        full_launch_btn.setStyleSheet("background-color: #4CAF50; color: white; padding: 10px;")
        full_launch_btn.clicked.connect(self.launch_complete_system)
        quick_layout.addWidget(full_launch_btn)
        
        # Camera test button
        camera_test_btn = QPushButton("📷 Launch Camera Test (No Motors)")
        camera_test_btn.setFont(QFont("Arial", 12))
        camera_test_btn.setStyleSheet("background-color: #2196F3; color: white; padding: 10px;")
        camera_test_btn.clicked.connect(self.launch_camera_test)
        quick_layout.addWidget(camera_test_btn)
        
        quick_group.setLayout(quick_layout)
        main_layout.addWidget(quick_group)

        # ==================== INDIVIDUAL COMPONENTS ====================
        components_group = QGroupBox("Individual Component Control")
        components_layout = QVBoxLayout()

        # Robot State Publisher (creates base_link)
        components_layout.addWidget(self.create_control_block(
            "robot_state",
            "Robot State Publisher (Creates base_link & Transforms)",
            """
cd ~/lunar_rover_ws
source install/setup.bash
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
    -p use_sim_time:=false
"""
        ))

        # Camera optical frame transforms
        components_layout.addWidget(self.create_control_block(
            "camera_transforms",
            "🔗 Camera Optical Frame Transforms",
            """
ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5707963267948966 0 -1.5707963267948966 camera_link camera_depth_optical_frame &
ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5707963267948966 0 -1.5707963267948966 camera_link camera_color_optical_frame &
ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5707963267948966 0 -1.5707963267948966 camera_rear_link camera_rear_color_optical_frame &
wait
"""
        ))

        # Motor controller
        components_layout.addWidget(self.create_control_block(
            "motor_controller",
            "🔧 Motor Controller (4-Wheel Drive)",
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

        # Front camera
        components_layout.addWidget(self.create_control_block(
            "front_camera",
            "📷 Front Camera (D435 - RGB + Depth + Point Cloud)",
            """
cd ~/lunar_rover_ws
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py \\
    camera_name:=camera \\
    camera_namespace:=camera \\
    enable_depth:=true \\
    enable_color:=true \\
    pointcloud.enable:=true \\
    align_depth.enable:=true
"""
        ))

        # Rear camera
        components_layout.addWidget(self.create_control_block(
            "rear_camera",
            "📷 Rear Camera (T265 - RGB Only)",
            """
cd ~/lunar_rover_ws
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py \\
    camera_name:=camera_rear \\
    camera_namespace:=camera_rear \\
    enable_color:=true \\
    enable_depth:=false
"""
        ))

        # Navigation
        components_layout.addWidget(self.create_control_block(
            "navigation",
            "🗺️ Unified Navigator (Point-Click + Obstacle Avoidance)",
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
            "👁️ RViz Visualization",
            """
cd ~/lunar_rover_ws
source install/setup.bash
ros2 run rviz2 rviz2 -d ~/lunar_rover_ws/src/lunar_robot_description/config/real_hardware_navigation.rviz \\
    --ros-args -p use_sim_time:=false
"""
        ))

        # Teleop
        components_layout.addWidget(self.create_control_block(
            "teleop",
            "⌨️ Keyboard Teleop (Simple - Drive + Camera)",
            """
cd ~/lunar_rover_ws
source install/setup.bash
python3 simple_teleop_keyboard.py
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
        """Launch the complete rover system using launch file"""
        self.status_lights["complete_system"] = StatusLight("yellow")
        proc = run_in_terminal(
            """
cd ~/lunar_rover_ws
source install/setup.bash
ros2 launch lunar_robot_hardware real_rover.launch.py
"""
        )
        self.processes["complete_system"] = proc

    def launch_camera_test(self):
        """Launch camera test mode"""
        self.status_lights["camera_test"] = StatusLight("yellow")
        proc = run_in_terminal(
            """
cd ~/lunar_rover_ws
source install/setup.bash
ros2 launch lunar_robot_hardware camera_test.launch.py
"""
        )
        self.processes["camera_test"] = proc

    def _format_port_config(self):
        """Format port configuration for display"""
        return f"FR: {self.motor_ports['front_right']} | FL: {self.motor_ports['front_left']} | " \
               f"BR: {self.motor_ports['back_right']} | BL: {self.motor_ports['back_left']} | " \
               f"CAM: {self.motor_ports['camera_rotation']}"

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