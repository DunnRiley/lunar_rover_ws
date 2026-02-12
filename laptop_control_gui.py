#!/usr/bin/env python3
"""
Simplified Laptop Control GUI
Controls for visualization and navigation only
Hardware control stays on mini PC
"""

import os
import signal
import sys
import subprocess
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QLabel,
    QHBoxLayout, QGroupBox, QTextEdit
)
from PyQt5.QtGui import QFont, QColor, QPainter
from PyQt5.QtCore import Qt, QTimer, QProcess


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


class LaptopControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Laptop Control - Rover Visualization & Navigation")
        self.setMinimumWidth(600)

        self.processes = {}
        self.status_lights = {}

        main_layout = QVBoxLayout()

        # HEADER
        header = QLabel("Laptop Control Panel")
        header.setFont(QFont("Arial", 16, QFont.Bold))
        header.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(header)

        # CONNECTION STATUS
        status_group = QGroupBox("Mini PC Connection")
        status_layout = QVBoxLayout()
        
        self.connection_label = QLabel("Checking connection...")
        status_layout.addWidget(self.connection_label)
        
        refresh_btn = QPushButton("Refresh Connection")
        refresh_btn.clicked.connect(self.check_connection)
        status_layout.addWidget(refresh_btn)
        
        status_group.setLayout(status_layout)
        main_layout.addWidget(status_group)

        # QUICK ACTIONS
        quick_group = QGroupBox("Quick Launch")
        quick_layout = QVBoxLayout()
        
        # Start All System
        all_btn = QPushButton("Start Complete System (Mini PC + Laptop)")
        all_btn.setFont(QFont("Arial", 12, QFont.Bold))
        all_btn.setStyleSheet("background-color: #4CAF50; color: white; padding: 10px;")
        all_btn.clicked.connect(self.start_complete_system)
        quick_layout.addWidget(all_btn)
        
        # Camera View Only
        camera_btn = QPushButton("Launch Camera View (RViz Only)")
        camera_btn.setFont(QFont("Arial", 11))
        camera_btn.setStyleSheet("background-color: #2196F3; color: white; padding: 8px;")
        camera_btn.clicked.connect(self.start_camera_view)
        quick_layout.addWidget(camera_btn)
        
        quick_group.setLayout(quick_layout)
        main_layout.addWidget(quick_group)

        # INDIVIDUAL CONTROLS
        controls_group = QGroupBox("Individual Controls")
        controls_layout = QVBoxLayout()
        
        # RViz
        controls_layout.addWidget(self.create_control_block(
            "rviz",
            "RViz Visualization",
            "ros2 run rviz2 rviz2 -d ~/lunar_rover_ws/hardware_navigation.rviz --ros-args -p use_sim_time:=false"
        ))
        
        # Keyboard Teleop
        controls_layout.addWidget(self.create_control_block(
            "teleop_keyboard",
            "Keyboard Teleop",
            "cd ~/lunar_rover_ws && python3 teleop_keyboard.py"
        ))
        
        # Controller Teleop
        controls_layout.addWidget(self.create_control_block(
            "teleop_controller",
            "Controller Teleop (Gamepad)",
            "cd ~/lunar_rover_ws && python3 controller_teleop.py"
        ))
        
        # Point-Click Navigation
        controls_layout.addWidget(self.create_control_block(
            "navigation",
            "Point-Click Navigation",
            "cd ~/lunar_rover_ws && ros2 run lunar_robot_autonomous unified_navigator"
        ))
        
        controls_group.setLayout(controls_layout)
        main_layout.addWidget(controls_group)

        # SYSTEM LOG
        log_group = QGroupBox("System Log")
        log_layout = QVBoxLayout()
        
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setMaximumHeight(150)
        log_layout.addWidget(self.log_display)
        
        log_group.setLayout(log_layout)
        main_layout.addWidget(log_group)

        # STOP ALL BUTTON
        stop_all_btn = QPushButton("STOP ALL PROCESSES")
        stop_all_btn.setFont(QFont("Arial", 14))
        stop_all_btn.setStyleSheet("background-color: #f44336; color: white; padding: 10px;")
        stop_all_btn.clicked.connect(self.stop_all_processes)
        main_layout.addWidget(stop_all_btn)

        self.setLayout(main_layout)

        # Timer for status updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_process_status)
        self.timer.start(500)
        
        # Check connection on startup
        QTimer.singleShot(1000, self.check_connection)
        
        self.log("Laptop Control GUI started")

    def log(self, message):
        """Add message to log display"""
        self.log_display.append(message)

    def check_connection(self):
        """Check connection to mini PC"""
        self.log("Checking connection to mini PC...")
        
        # Try to list topics
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True,
            text=True,
            timeout=3
        )
        
        if '/camera/camera/color/image_raw' in result.stdout:
            self.connection_label.setText("✓ Connected to Mini PC (camera topics visible)")
            self.connection_label.setStyleSheet("color: green; font-weight: bold;")
            self.log("✓ Connected to mini PC")
        else:
            self.connection_label.setText("✗ Not connected (no camera topics)")
            self.connection_label.setStyleSheet("color: red; font-weight: bold;")
            self.log("✗ Cannot see mini PC topics. Make sure mini PC is running.")

    def start_complete_system(self):
        """Start complete system: mini PC + laptop viz"""
        self.log("Starting complete system...")
        
        # Start mini PC via SSH
        self.log("→ Starting mini PC hardware via SSH...")
        minipc_cmd = "ssh moonpie@138.67.181.222 'bash ~/lunar_rover_ws/mini_pc_launch.sh'"
        subprocess.Popen(minipc_cmd, shell=True)
        
        # Wait for mini PC to start
        self.log("  Waiting 5 seconds for mini PC...")
        QTimer.singleShot(5000, self.start_camera_view)

    def start_camera_view(self):
        """Start just RViz for camera viewing"""
        self.log("Starting RViz camera view...")
        self.start_process(
            "rviz",
            "ros2 run rviz2 rviz2 -d ~/lunar_rover_ws/hardware_navigation.rviz --ros-args -p use_sim_time:=false"
        )

    def create_control_block(self, name, label_text, command):
        """Create control group with start/stop buttons"""
        group = QGroupBox(label_text)
        layout = QHBoxLayout()

        light = StatusLight("red")
        self.status_lights[name] = light
        layout.addWidget(light)

        btn_start = QPushButton("Start")
        btn_start.clicked.connect(lambda: self.start_process(name, command))
        layout.addWidget(btn_start)

        btn_stop = QPushButton("Stop")
        btn_stop.clicked.connect(lambda: self.stop_process(name))
        layout.addWidget(btn_stop)

        group.setLayout(layout)
        return group

    def start_process(self, name, command):
        """Start a process using QProcess"""
        if name in self.processes and self.processes[name] is not None:
            self.log(f"⚠ {name} already running, stopping first...")
            self.stop_process(name)

        self.log(f"Starting {name}...")
        self.status_lights[name].set_color("yellow")
        
        process = QProcess()
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyRead.connect(lambda: self.handle_output(name, process))
        process.start("bash", ["-c", command])
        
        self.processes[name] = process
        
        if process.state() == QProcess.Running:
            self.log(f"✓ {name} started")
            self.status_lights[name].set_color("green")
        else:
            self.log(f"✗ Failed to start {name}")
            self.status_lights[name].set_color("red")

    def handle_output(self, name, process):
        """Handle process output"""
        output = bytes(process.readAll()).decode('utf-8').strip()
        if output:
            self.log(f"[{name}] {output[:100]}")  # Limit output length

    def stop_process(self, name):
        """Stop a process"""
        proc = self.processes.get(name)
        if proc is not None and proc.state() == QProcess.Running:
            self.log(f"Stopping {name}...")
            proc.terminate()
            proc.waitForFinished(3000)
            if proc.state() == QProcess.Running:
                proc.kill()
            self.processes[name] = None
            self.status_lights[name].set_color("red")
            self.log(f"✓ {name} stopped")

    def stop_all_processes(self):
        """Emergency stop all processes"""
        self.log("STOPPING ALL PROCESSES...")
        for name in list(self.processes.keys()):
            self.stop_process(name)
        self.log("✓ All processes stopped")

    def update_process_status(self):
        """Update status lights based on process state"""
        for name, proc in self.processes.items():
            if proc is None or proc.state() != QProcess.Running:
                self.status_lights[name].set_color("red")
            else:
                self.status_lights[name].set_color("green")


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = LaptopControlGUI()
    window.show()
    sys.exit(app.exec_())