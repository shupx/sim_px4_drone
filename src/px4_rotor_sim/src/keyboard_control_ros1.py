#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Advanced Drone Control Interface for PX4 Autopilot

This PyQt5-based GUI provides real-time control of a PX4-based drone via ROS1 MAVROS.
It supports keyboard and mouse input for velocity-based control with position hold for altitude.

Features:
- Real-time keyboard/mouse control (WASD for translation, QE for altitude, ZC for yaw)
- Automatic arming and OFFBOARD mode activation
- Dynamic velocity parameter adjustment with sliders
- Real-time publish rate monitoring (10Hz)
- Command history and debug logging
- Position target control (xy velocity, z altitude, yaw rate)
- World frame (NED) control with masked position targets

Requirements:
- ROS1 with MAVROS running
- PX4 sim node
- PyQt5 for GUI

Author: Peixuan Shu (Beihang University)
"""

import sys
import rospy
import threading
import time
import json
import os
from datetime import datetime
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, 
    QHBoxLayout, QPushButton, QGridLayout, QLabel, 
    QSlider, QSpinBox, QDoubleSpinBox, QTextEdit,
    QTabWidget, QGroupBox
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QSize, QPoint
from PyQt5.QtGui import QFont, QColor, QPalette, QCursor
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
import numpy as np


class ControlSignals(QObject):
    """Signal dispatcher for thread-safe communication between ROS backend and PyQt5 UI"""
    state_updated = pyqtSignal(dict)
    log_message = pyqtSignal(str)
    

class DroneController(QObject):
    """ROS1 backend controller for PX4 drone via MAVROS"""
    
    def __init__(self, signals):
        super().__init__()
        self.signals = signals
        
        # Initialize ROS node
        try:
            rospy.init_node('drone_control_ui_advanced', anonymous=True)
        except rospy.exceptions.ROSException:
            # Node already initialized, continue
            pass
        
        # Publisher and Subscriber
        self.setpoint_pub = rospy.Publisher(
            'mavros/setpoint_raw/local',
            PositionTarget,
            queue_size=10
        )
        self.state_sub = rospy.Subscriber(
            'mavros/state',
            State,
            self.state_callback
        )
        
        # ROS Service Clients (non-blocking)
        self.arming_client = None
        self.set_mode_client = None
        self.services_ready = False
        
        # Drone state
        self.current_state = State()
        self.is_armed = False
        self.is_offboard = False
        
        # Velocity and position control
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        
        # Publishing thread flag (must initialize before starting thread)
        self.running = True
        
        # Start service connection thread
        self.service_thread = threading.Thread(target=self.connect_services, daemon=True)
        self.service_thread.start()
        
        # Statistics
        self.publish_count = 0
        self.last_publish_time = time.time()
        self.publish_rate = 0.0
        
        # Publishing thread
        self.publish_thread = threading.Thread(target=self.publish_loop, daemon=True)
        self.publish_thread.start()
        
        self.signals.log_message.emit(f"[{datetime.now().strftime('%H:%M:%S')}] Controller initialized successfully")
        
    def connect_services(self):
        """Try to connect to MAVROS services (background thread)"""
        while self.running and not rospy.is_shutdown():
            try:
                if not self.services_ready:
                    rospy.wait_for_service('mavros/cmd/arming', timeout=1.0)
                    rospy.wait_for_service('mavros/set_mode', timeout=1.0)
                    self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
                    self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
                    self.services_ready = True
                    msg = f"[{datetime.now().strftime('%H:%M:%S')}] ✓ MAVROS services connected"
                    self.signals.log_message.emit(msg)
                    rospy.loginfo(msg)
                    break
            except (rospy.ServiceException, rospy.ROSException, rospy.exceptions.ROSException):
                # Retry after 1 second
                time.sleep(1)
        
    def state_callback(self, msg):
        """State subscriber callback"""
        self.current_state = msg
        self.is_armed = msg.armed
        self.is_offboard = msg.mode == "OFFBOARD"
        
        # Emit signal to update UI
        self.signals.state_updated.emit({
            'armed': self.is_armed,
            'offboard': self.is_offboard,
            'mode': msg.mode,
            'publish_rate': self.publish_rate
        })
    
    def set_offboard_mode(self):
        """Switch to OFFBOARD mode and auto-arm if needed"""
        try:
            if not self.is_offboard:
                # Check if services are ready
                if not self.services_ready or self.set_mode_client is None or self.arming_client is None:
                    msg = f"[{datetime.now().strftime('%H:%M:%S')}] ⚠ MAVROS services not connected yet, skipping OFFBOARD switch"
                    self.signals.log_message.emit(msg)
                    rospy.logwarn(msg)
                    return False
                
                # Step 1: Arm the drone if not armed
                if not self.is_armed:
                    msg = f"[{datetime.now().strftime('%H:%M:%S')}] → Arming drone..."
                    self.signals.log_message.emit(msg)
                    rospy.loginfo(msg)
                    
                    try:
                        response = self.arming_client(True)
                        if response.success:
                            # Wait for actual armed state
                            start_time = time.time()
                            while not self.is_armed and time.time() - start_time < 5.0:
                                time.sleep(0.1)
                            
                            if self.is_armed:
                                msg = f"[{datetime.now().strftime('%H:%M:%S')}] ✓ Drone armed successfully"
                                self.signals.log_message.emit(msg)
                                rospy.loginfo(msg)
                            else:
                                msg = f"[{datetime.now().strftime('%H:%M:%S')}] ✗ Arming timeout, no armed state received"
                                self.signals.log_message.emit(msg)
                                rospy.logwarn(msg)
                                return False
                        else:
                            msg = f"[{datetime.now().strftime('%H:%M:%S')}] ✗ Arming command rejected"
                            self.signals.log_message.emit(msg)
                            rospy.logwarn(msg)
                            return False
                    except Exception as e:
                        msg = f"[{datetime.now().strftime('%H:%M:%S')}] ✗ Arming exception: {e}"
                        self.signals.log_message.emit(msg)
                        rospy.logerr(msg)
                        return False
                
                # Step 2: Send setpoint messages to prepare flight controller
                msg = f"[{datetime.now().strftime('%H:%M:%S')}] → Sending setpoint messages..."
                self.signals.log_message.emit(msg)
                for _ in range(10):
                    self.publish_setpoint()
                    time.sleep(0.05)
                
                # Step 3: Switch to OFFBOARD mode
                msg = f"[{datetime.now().strftime('%H:%M:%S')}] → Switching to OFFBOARD mode..."
                self.signals.log_message.emit(msg)
                rospy.loginfo(msg)
                
                try:
                    response = self.set_mode_client(custom_mode="OFFBOARD")
                    if response.mode_sent:
                        # Wait for actual OFFBOARD state
                        start_time = time.time()
                        while not self.is_offboard and time.time() - start_time < 5.0:
                            time.sleep(0.1)
                        
                        if self.is_offboard:
                            msg = f"[{datetime.now().strftime('%H:%M:%S')}] ✓ Switched to OFFBOARD mode successfully"
                            self.signals.log_message.emit(msg)
                            rospy.loginfo(msg)
                            return True
                        else:
                            msg = f"[{datetime.now().strftime('%H:%M:%S')}] ✗ Mode switch timeout, no OFFBOARD state received"
                            self.signals.log_message.emit(msg)
                            rospy.logwarn(msg)
                            return False
                    else:
                        msg = f"[{datetime.now().strftime('%H:%M:%S')}] ✗ OFFBOARD mode switch failed"
                        self.signals.log_message.emit(msg)
                        return False
                except Exception as e:
                    msg = f"[{datetime.now().strftime('%H:%M:%S')}] ✗ Mode switch exception: {e}"
                    self.signals.log_message.emit(msg)
                    rospy.logerr(msg)
                    return False
            
            return True
        except Exception as e:
            msg = f"[{datetime.now().strftime('%H:%M:%S')}] ✗ Exception: {e}"
            self.signals.log_message.emit(msg)
            rospy.logerr(msg)
            return False
    
    def set_velocity(self, vx, vy, vz, yaw_rate):
        """Set velocity commands"""
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.yaw_rate = yaw_rate
    
    def publish_setpoint(self):
        """Publish setpoint message using PositionTarget"""
        # Integrate height from vertical velocity
        dt = 0.1  # 10Hz
        self.current_z += self.vz * dt
        self.current_yaw += self.yaw_rate * dt
        
        # Create PositionTarget message (world frame)
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"  # World frame
        msg.coordinate_frame = 1  # FRAME_LOCAL_NED
        
        # Set velocities (xy only) and position (z altitude), yaw rate
        msg.velocity.x = self.vx
        msg.velocity.y = self.vy
        msg.velocity.z = 0.0  # z velocity unused
        msg.position.z = self.current_z  # Absolute altitude
        msg.yaw_rate = self.yaw_rate
        
        # Bitmask: send xy velocity, z altitude, yaw_rate only
        msg.type_mask = 0b010111000011 
        
        self.setpoint_pub.publish(msg)
        self.publish_count += 1
    
    def publish_loop(self):
        """Publishing loop at 10Hz"""
        rate = rospy.Rate(10)
        while self.running and not rospy.is_shutdown():
            try:
                if self.is_offboard:
                    self.publish_setpoint()
                    
                    # Calculate publish rate
                    current_time = time.time()
                    if current_time - self.last_publish_time >= 1.0:
                        self.publish_rate = self.publish_count
                        self.publish_count = 0
                        self.last_publish_time = current_time
                        
                rate.sleep()
            except Exception as e:
                rospy.logerr(f"Error in publish loop: {e}")
    
    def stop(self):
        """Stop the controller"""
        self.running = False
        self.publish_thread.join(timeout=1)


class ControlButton(QPushButton):
    """Custom control button with active/inactive states"""
    
    def __init__(self, label):
        super().__init__(label)
        self.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                border-radius: 2px;
                font-weight: bold;
                font-size: 11px;
                padding: 0px;
                margin: 0px;
            }
            QPushButton:pressed {
                background-color: #45a049;
                border: 1px solid #333;
            }
        """)
        self.is_active = False
        from PyQt5.QtWidgets import QSizePolicy
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
    
    def set_active(self, active):
        """Set button active state"""
        self.is_active = active
        if active:
            self.setStyleSheet("""
                QPushButton {
                    background-color: #f44336;
                    color: white;
                    border: 1px solid #333;
                    border-radius: 2px;
                    font-weight: bold;
                    font-size: 11px;
                    padding: 1px;
                }
            """)
        else:
            self.setStyleSheet("""
                QPushButton {
                    background-color: #4CAF50;
                    color: white;
                    border: none;
                    border-radius: 2px;
                    font-weight: bold;
                    font-size: 11px;
                    padding: 1px;
                }
            """)


class DroneControlUIAdvanced(QMainWindow):
    """Advanced drone control UI with PyQt5"""
    
    def __init__(self):
        super().__init__()
        
        # Create signal dispatcher
        self.signals = ControlSignals()
        
        # Create ROS controller
        self.controller = DroneController(self.signals)
        
        # Connect signals
        self.controller.signals.state_updated.connect(self.on_state_updated)
        self.controller.signals.log_message.connect(self.on_log_message)
        
        # Get config file path (same directory as this script)
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_file = os.path.join(self.script_dir, 'drone_control_config.json')
        
        # Control parameters - load from config if available
        self.speed = 1.0
        self.yaw_speed = 1.57
        self.config_load_msg = ""  # Will store config load message for later display
        self.load_config()
        
        # Button states
        self.button_states = {
            'up': False,
            'down': False,
            'forward': False,
            'backward': False,
            'left': False,
            'right': False,
            'yaw_left': False,
            'yaw_right': False,
            'recover': False
        }
        
        # Set initial altitude to 1m
        self.controller.current_z = 1.0
        
        self.init_ui()
        self.setWindowTitle("Drone Control Interface")
        self.resize(250, 300)
        
        # Position window at mouse cursor
        cursor_pos = QCursor.pos()
        self.move(cursor_pos.x(), cursor_pos.y())
        
        # Display config load message in debug log (now that log_text exists)
        if self.config_load_msg:
            self.on_log_message(self.config_load_msg)
        
        # Auto-start OFFBOARD mode after 2 seconds
        QTimer.singleShot(2000, self.auto_start_offboard)
        
    def init_ui(self):
        """Initialize UI"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(1, 1, 1, 1)
        main_layout.setSpacing(0)
        
        # Create tabs
        tabs = QTabWidget()
        
        # Tab 1: Control Panel
        control_tab = self.create_control_tab()
        tabs.addTab(control_tab, "Control Panel")
        
        # Tab 2: Parameters
        params_tab = self.create_params_tab()
        tabs.addTab(params_tab, "Parameters")
        
        # Tab 3: Debug Info
        debug_tab = self.create_debug_tab()
        tabs.addTab(debug_tab, "Debug Info")
        
        main_layout.addWidget(tabs)
        central_widget.setLayout(main_layout)
        
        # Update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_control)
        self.update_timer.start(100)  # 10Hz update
    
    def auto_start_offboard(self):
        """Auto-start OFFBOARD mode (called after UI initialization)"""
        self.controller.set_offboard_mode()
    
    def create_control_tab(self):
        """Create control panel tab"""
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setContentsMargins(1, 1, 1, 1)
        layout.setSpacing(0)
        
        # Status label
        self.status_label = QLabel("Status: Initializing, waiting for MAVROS connection...")
        self.status_label.setStyleSheet("color: orange; font-weight: bold; font-size: 12px;")
        self.status_label.setMaximumHeight(25)
        layout.addWidget(self.status_label)
        
        # Keyboard instructions
        info_label = QLabel("W/A/S/D Move | Q/E Altitude | Z/C Yaw | Space Stop")
        info_label.setStyleSheet("font-size: 11px; color: #666;")
        info_label.setMaximumHeight(20)
        layout.addWidget(info_label)
        
        # Main control area - arranged by keyboard layout
        control_layout = QGridLayout()
        control_layout.setSpacing(3)
        control_layout.setContentsMargins(2, 2, 2, 2)
        
        # Set uniform row/column stretch
        for i in range(3):
            control_layout.setRowStretch(i, 1)
        for j in range(3):
            control_layout.setColumnStretch(j, 1)
        
        # Row 1: Q W E (Up/Forward/Down)
        self.btn_up = ControlButton("Q\nUp\nz+")
        self.btn_forward = ControlButton("W\nForward\nx+")
        self.btn_down = ControlButton("E\nDown\nz-")
        
        control_layout.addWidget(self.btn_up, 0, 0)
        control_layout.addWidget(self.btn_forward, 0, 1)
        control_layout.addWidget(self.btn_down, 0, 2)
        
        # Row 2: A S D (Left/Backward/Right)
        self.btn_left = ControlButton("A\nLeft\ny+")
        self.btn_backward = ControlButton("S\nBackward\nx-")
        self.btn_right = ControlButton("D\nRight\ny-")
        
        control_layout.addWidget(self.btn_left, 1, 0)
        control_layout.addWidget(self.btn_backward, 1, 1)
        control_layout.addWidget(self.btn_right, 1, 2)
        
        # Row 3: Z Space C (Yaw Left/Stop/Yaw Right)
        self.btn_yaw_left = ControlButton("Z\nYaw L")
        self.btn_recover = ControlButton("Space\nStop")
        self.btn_yaw_right = ControlButton("C\nYaw R")
        
        control_layout.addWidget(self.btn_yaw_left, 2, 0)
        control_layout.addWidget(self.btn_recover, 2, 1)
        control_layout.addWidget(self.btn_yaw_right, 2, 2)
        
        layout.addLayout(control_layout, 1)  # Add stretch factor
        
        # Connect button events - using clicked signal for toggle
        self.btn_up.clicked.connect(lambda: self.button_pressed('up'))
        self.btn_down.clicked.connect(lambda: self.button_pressed('down'))
        self.btn_forward.clicked.connect(lambda: self.button_pressed('forward'))
        self.btn_backward.clicked.connect(lambda: self.button_pressed('backward'))
        self.btn_left.clicked.connect(lambda: self.button_pressed('left'))
        self.btn_right.clicked.connect(lambda: self.button_pressed('right'))
        self.btn_yaw_left.clicked.connect(lambda: self.button_pressed('yaw_left'))
        self.btn_yaw_right.clicked.connect(lambda: self.button_pressed('yaw_right'))
        self.btn_recover.clicked.connect(self.recover)
        
        widget.setLayout(layout)
        return widget
    
    def create_params_tab(self):
        """Create parameter adjustment tab"""
        widget = QWidget()
        layout = QVBoxLayout()
        
        # Linear velocity
        speed_group = QGroupBox("Linear Velocity (m/s)")
        speed_layout = QHBoxLayout()
        self.speed_spinbox = QDoubleSpinBox()
        self.speed_spinbox.setRange(0.0, 10.0)
        self.speed_spinbox.setValue(self.speed)
        self.speed_spinbox.setSingleStep(0.1)
        self.speed_spinbox.valueChanged.connect(self.on_speed_changed)
        speed_layout.addWidget(self.speed_spinbox)
        
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(0, 100)
        self.speed_slider.setValue(int(self.speed * 10))
        self.speed_slider.setTickPosition(QSlider.TicksBelow)
        self.speed_slider.setTickInterval(10)
        self.speed_slider.sliderMoved.connect(self.on_speed_slider_changed)
        speed_layout.addWidget(self.speed_slider)
        
        speed_group.setLayout(speed_layout)
        layout.addWidget(speed_group)
        
        # Angular velocity
        yaw_group = QGroupBox("Angular Velocity (rad/s)")
        yaw_layout = QHBoxLayout()
        self.yaw_spinbox = QDoubleSpinBox()
        self.yaw_spinbox.setRange(0.0, 6.28)
        self.yaw_spinbox.setValue(self.yaw_speed)
        self.yaw_spinbox.setSingleStep(0.1)
        self.yaw_spinbox.valueChanged.connect(self.on_yaw_changed)
        yaw_layout.addWidget(self.yaw_spinbox)
        
        self.yaw_slider = QSlider(Qt.Horizontal)
        self.yaw_slider.setRange(0, 628)
        self.yaw_slider.setValue(int(self.yaw_speed * 100))
        self.yaw_slider.setTickPosition(QSlider.TicksBelow)
        self.yaw_slider.setTickInterval(100)
        self.yaw_slider.sliderMoved.connect(self.on_yaw_slider_changed)
        yaw_layout.addWidget(self.yaw_slider)
        
        yaw_group.setLayout(yaw_layout)
        layout.addWidget(yaw_group)
        
        # Save config button
        save_btn = QPushButton("Save Config")
        save_btn.clicked.connect(self.save_config)
        layout.addWidget(save_btn)
        
        layout.addStretch()
        widget.setLayout(layout)
        return widget
    
    def create_debug_tab(self):
        """Create debug info tab"""
        widget = QWidget()
        layout = QVBoxLayout()
        
        # Log text area
        layout.addWidget(QLabel("Log Messages:"))
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet("background-color: #f5f5f5; font-family: monospace;")
        layout.addWidget(self.log_text)
        
        # Clear log button
        clear_btn = QPushButton("Clear Log")
        clear_btn.clicked.connect(self.log_text.clear)
        layout.addWidget(clear_btn)
        
        widget.setLayout(layout)
        return widget
    
    def on_speed_changed(self, value):
        """Linear velocity value changed"""
        self.speed = value
        self.speed_slider.blockSignals(True)
        self.speed_slider.setValue(int(value * 10))
        self.speed_slider.blockSignals(False)
    
    def on_speed_slider_changed(self, value):
        """Linear velocity slider changed"""
        self.speed = value / 10.0
        self.speed_spinbox.blockSignals(True)
        self.speed_spinbox.setValue(self.speed)
        self.speed_spinbox.blockSignals(False)
    
    def on_yaw_changed(self, value):
        """Angular velocity value changed"""
        self.yaw_speed = value
        self.yaw_slider.blockSignals(True)
        self.yaw_slider.setValue(int(value * 100))
        self.yaw_slider.blockSignals(False)
    
    def on_yaw_slider_changed(self, value):
        """Angular velocity slider changed"""
        self.yaw_speed = value / 100.0
        self.yaw_spinbox.blockSignals(True)
        self.yaw_spinbox.setValue(self.yaw_speed)
        self.yaw_spinbox.blockSignals(False)
    
    def load_config(self):
        """Load configuration from file if it exists"""
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    config = json.load(f)
                self.speed = config.get('speed', 2.0)
                self.yaw_speed = config.get('yaw_speed', 1.57)
                msg = f"[{datetime.now().strftime('%H:%M:%S')}] ✓ Config loaded from {self.config_file}"
                rospy.loginfo(msg)
                # Store for later UI display after log_text is initialized
                self.config_load_msg = msg
            except Exception as e:
                msg = f"[{datetime.now().strftime('%H:%M:%S')}] ✗ Failed to load config: {e}, using defaults"
                rospy.logwarn(msg)
                self.config_load_msg = msg
        else:
            msg = f"[{datetime.now().strftime('%H:%M:%S')}] ℹ No config file found at {self.config_file}, using defaults"
            rospy.loginfo(msg)
            self.config_load_msg = msg
    
    def save_config(self):
        """Save configuration to file"""
        try:
            config = {
                'speed': self.speed,
                'yaw_speed': self.yaw_speed,
                'timestamp': datetime.now().isoformat()
            }
            with open(self.config_file, 'w') as f:
                json.dump(config, f, indent=2)
            msg = f"[{datetime.now().strftime('%H:%M:%S')}] ✓ Config saved to {self.config_file}"
            self.on_log_message(msg)
        except Exception as e:
            msg = f"[{datetime.now().strftime('%H:%M:%S')}] ✗ Failed to save config: {e}"
            self.on_log_message(msg)
    
    def button_pressed(self, button_name):
        """Button pressed - toggle button state"""
        # Mouse click toggles button state (not hold)
        self.button_states[button_name] = not self.button_states[button_name]
        self.update_buttons_display()
        self.update_control()
    
    def button_released(self, button_name):
        """Button released - no action needed"""
        pass
    
    def recover(self):
        """Stop all movement"""
        for key in self.button_states:
            if key != 'recover':
                self.button_states[key] = False
        self.update_buttons_display()
        self.update_control()
    
    def keyPressEvent(self, event):
        """Keyboard press event"""
        if event.isAutoRepeat():
            return
        
        key = event.key()
        
        if key == Qt.Key_W:
            self.button_states['forward'] = True
        elif key == Qt.Key_S:
            self.button_states['backward'] = True
        elif key == Qt.Key_A:
            self.button_states['left'] = True
        elif key == Qt.Key_D:
            self.button_states['right'] = True
        elif key == Qt.Key_Q:
            self.button_states['up'] = True
        elif key == Qt.Key_E:
            self.button_states['down'] = True
        elif key == Qt.Key_Z:
            self.button_states['yaw_left'] = True
        elif key == Qt.Key_C:
            self.button_states['yaw_right'] = True
        elif key == Qt.Key_Space:
            self.recover()
            return
        
        self.update_buttons_display()
        self.update_control()
    
    def keyReleaseEvent(self, event):
        """Keyboard release event"""
        if event.isAutoRepeat():
            return
        
        key = event.key()
        
        if key == Qt.Key_W:
            self.button_states['forward'] = False
        elif key == Qt.Key_S:
            self.button_states['backward'] = False
        elif key == Qt.Key_A:
            self.button_states['left'] = False
        elif key == Qt.Key_D:
            self.button_states['right'] = False
        elif key == Qt.Key_Q:
            self.button_states['up'] = False
        elif key == Qt.Key_E:
            self.button_states['down'] = False
        elif key == Qt.Key_Z:
            self.button_states['yaw_left'] = False
        elif key == Qt.Key_C:
            self.button_states['yaw_right'] = False
        
        self.update_buttons_display()
        self.update_control()
    
    def update_buttons_display(self):
        """Update button display"""
        self.btn_up.set_active(self.button_states['up'])
        self.btn_down.set_active(self.button_states['down'])
        self.btn_forward.set_active(self.button_states['forward'])
        self.btn_backward.set_active(self.button_states['backward'])
        self.btn_left.set_active(self.button_states['left'])
        self.btn_right.set_active(self.button_states['right'])
        self.btn_yaw_left.set_active(self.button_states['yaw_left'])
        self.btn_yaw_right.set_active(self.button_states['yaw_right'])
    
    def update_control(self):
        """Update control velocities"""
        if not self.controller.is_offboard:
            if any([v for k, v in self.button_states.items() if k != 'recover']):
                self.controller.set_offboard_mode()
        
        vx = 0.0
        vy = 0.0
        vz = 0.0
        yaw_rate = 0.0
        
        if self.button_states['forward']:
            vx += self.speed
        if self.button_states['backward']:
            vx -= self.speed
        
        if self.button_states['left']:
            vy += self.speed
        if self.button_states['right']:
            vy -= self.speed
        
        if self.button_states['up']:
            vz += self.speed
        if self.button_states['down']:
            vz -= self.speed
        
        if self.button_states['yaw_left']:
            yaw_rate += self.yaw_speed
        if self.button_states['yaw_right']:
            yaw_rate -= self.yaw_speed
        
        self.controller.set_velocity(vx, vy, vz, yaw_rate)
    
    def on_state_updated(self, state_dict):
        """State update callback"""
        armed = state_dict.get('armed', False)
        offboard = state_dict.get('offboard', False)
        mode = state_dict.get('mode', 'UNKNOWN')
        pub_rate = state_dict.get('publish_rate', 0)
        
        if armed and offboard:
            status_text = f"✓ OFFBOARD | Armed | Rate: {pub_rate}Hz"
            color = "green"
        elif armed:
            status_text = f"⚠ {mode} | Armed | Rate: {pub_rate}Hz"
            color = "orange"
        else:
            status_text = f"✗ {mode} | Disarmed | Rate: {pub_rate}Hz"
            color = "red"
        
        self.status_label.setText(f"Status: {status_text}")
        self.status_label.setStyleSheet(f"color: {color}; font-weight: bold; font-size: 12px;")
    
    def on_log_message(self, message):
        """Log message callback"""
        self.log_text.append(message)
    
    def closeEvent(self, event):
        """Close event"""
        self.update_timer.stop()
        self.controller.stop()
        event.accept()


def main():
    """Main function"""
    try:
        app = QApplication(sys.argv)
        ui = DroneControlUIAdvanced()
        ui.show()
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        rospy.loginfo("Program interrupted by user")
    except Exception as e:
        rospy.logerr(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
