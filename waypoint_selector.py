#!/usr/bin/env python3
"""
Interactive Waypoint Selector
Click on point cloud to select destinations, track distance using odometry
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import math

class WaypointSelector(Node):
    def __init__(self):
        super().__init__('waypoint_selector')
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        self.target_pub = self.create_publisher(Marker, '/target_marker', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.clicked_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )
        
        # State
        self.current_pose = None
        self.waypoints = []
        self.current_target = None
        
        # Timer for updates
        self.create_timer(0.5, self.update_display)
        
        print("\n" + "="*70)
        print("  Interactive Waypoint Selector")
        print("="*70)
        print("\nWaiting for odometry...")
    
    def odom_callback(self, msg):
        """Update current robot position"""
        self.current_pose = msg.pose.pose
        
        # Print distance to target if we have one
        if self.current_target is not None:
            dist = self.distance_to_target()
            if dist < 0.5:
                print(f"\r✓ REACHED TARGET! Distance: {dist:.2f}m", end='')
            else:
                print(f"\rDistance to target: {dist:.2f}m   ", end='', flush=True)
    
    def clicked_point_callback(self, msg):
        """Handle clicked points from RViz"""
        point = msg.point
        
        print(f"\n\n→ Clicked point: ({point.x:.2f}, {point.y:.2f}, {point.z:.2f})")
        
        # Ask what to do with this point
        print("\nWhat do you want to do?")
        print("  1. Add as waypoint")
        print("  2. Set as current target")
        print("  3. Ignore")
        
        choice = input("Choice (1/2/3): ")
        
        if choice == '1':
            desc = input("Description: ") or f"Waypoint {len(self.waypoints) + 1}"
            self.waypoints.append({'point': point, 'desc': desc})
            print(f"✓ Added waypoint: {desc}")
            self.publish_markers()
        
        elif choice == '2':
            self.current_target = point
            print(f"✓ Target set! Drive towards ({point.x:.2f}, {point.y:.2f})")
            self.publish_target()
        
        self.show_status()
    
    def distance_to_target(self):
        """Calculate distance from current position to target"""
        if self.current_pose is None or self.current_target is None:
            return float('inf')
        
        dx = self.current_target.x - self.current_pose.position.x
        dy = self.current_target.y - self.current_pose.position.y
        dz = self.current_target.z - self.current_pose.position.z
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def publish_markers(self):
        """Publish waypoint markers for RViz"""
        marker_array = MarkerArray()
        
        for i, wp in enumerate(self.waypoints):
            # Sphere marker
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position = wp['point']
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
            
            # Text marker
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "waypoint_labels"
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position = wp['point']
            text_marker.pose.position.z += 0.3
            
            text_marker.scale.z = 0.2
            
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = wp['desc']
            
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)
    
    def publish_target(self):
        """Publish current target marker"""
        if self.current_target is None:
            return
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Arrow pointing down at target
        marker.pose.position = self.current_target
        marker.pose.position.z += 0.5
        marker.pose.orientation.x = 0.707
        marker.pose.orientation.w = 0.707
        
        marker.scale.x = 0.5
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.9
        
        self.target_pub.publish(marker)
    
    def update_display(self):
        """Periodically update markers"""
        if self.waypoints:
            self.publish_markers()
        if self.current_target:
            self.publish_target()
    
    def show_status(self):
        """Print current status"""
        print("\n" + "="*70)
        print(f"  Waypoints: {len(self.waypoints)}")
        for i, wp in enumerate(self.waypoints, 1):
            p = wp['point']
            print(f"    {i}. {wp['desc']}: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")
        
        if self.current_target:
            p = self.current_target
            print(f"\n  Current Target: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")
            if self.current_pose:
                print(f"  Distance: {self.distance_to_target():.2f}m")
        else:
            print("\n  No target set")
        
        print("="*70)
        print("\nIn RViz: Click 'Publish Point' tool, then click on the point cloud")
        print()

def main():
    print("\n" + "="*70)
    print("  Interactive Waypoint Selector")
    print("="*70)
    print("\nThis tool lets you:")
    print("  • Click on the 3D point cloud to select destinations")
    print("  • Track distance to target using visual odometry")
    print("  • Save multiple waypoints")
    print("\nMake sure RViz is running with:")
    print("  • Point cloud display (/rtabmap/cloud_map)")
    print("  • 'Publish Point' tool enabled")
    print("="*70)
    
    input("\nPress Enter to start...")
    
    rclpy.init()
    node = WaypointSelector()
    
    print("\n✓ Ready! Waiting for you to click points in RViz...")
    node.show_status()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nSaving waypoints...")
        
        # Save to file
        with open('/tmp/waypoints.txt', 'w') as f:
            for wp in node.waypoints:
                p = wp['point']
                f.write(f"{wp['desc']},{p.x},{p.y},{p.z}\n")
        
        if node.waypoints:
            print(f"✓ Saved {len(node.waypoints)} waypoints to /tmp/waypoints.txt")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # Import here to show nice error if missing
    try:
        from geometry_msgs.msg import PointStamped
    except ImportError:
        print("ERROR: Missing geometry_msgs")
        print("Install: sudo apt install ros-jazzy-geometry-msgs")
        exit(1)
    
    main()