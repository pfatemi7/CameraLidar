#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Performance Monitor Node for Camera-LiDAR Fusion System
Provides real-time performance metrics, parameter tuning, and system health monitoring.
"""

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, PointCloud2, Float32MultiArray
from std_msgs.msg import String, Header, Bool
from cv_bridge import CvBridge
import struct
import time
import threading
from collections import deque, defaultdict
import json
import os
from datetime import datetime
import psutil
import subprocess

class PerformanceMonitorNode:
    def __init__(self):
        rospy.init_node('performance_monitor_node', anonymous=True)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Publishers
        self.performance_image_pub = rospy.Publisher('/monitor/performance_image', Image, queue_size=1)
        self.system_stats_pub = rospy.Publisher('/monitor/system_stats', Float32MultiArray, queue_size=1)
        self.health_status_pub = rospy.Publisher('/monitor/health_status', String, queue_size=1)
        self.parameter_suggestions_pub = rospy.Publisher('/monitor/parameter_suggestions', String, queue_size=1)
        
        # Subscribers
        self.filtering_stats_sub = rospy.Subscriber('/fused/filtering_stats', Float32MultiArray, 
                                                   self.filtering_stats_callback)
        self.detection_stats_sub = rospy.Subscriber('/detection/stats', Float32MultiArray, 
                                                   self.detection_stats_callback)
        self.calibration_quality_sub = rospy.Subscriber('/calibration/quality', Float32MultiArray, 
                                                       self.calibration_quality_callback)
        
        # Performance monitoring
        self.filtering_stats = deque(maxlen=100)
        self.detection_stats = deque(maxlen=100)
        self.calibration_stats = deque(maxlen=100)
        self.system_stats = deque(maxlen=100)
        
        # Performance thresholds
        self.target_fps = rospy.get_param('~target_fps', 5.0)
        self.max_processing_time = rospy.get_param('~max_processing_time', 0.2)  # seconds
        self.max_memory_usage = rospy.get_param('~max_memory_usage', 80.0)  # percentage
        self.max_cpu_usage = rospy.get_param('~max_cpu_usage', 90.0)  # percentage
        
        # Parameter optimization
        self.parameter_history = defaultdict(list)
        self.performance_history = defaultdict(list)
        self.optimization_active = False
        
        # System monitoring
        self.start_time = time.time()
        self.frame_count = 0
        self.last_stats_time = time.time()
        
        # Performance metrics
        self.current_fps = 0.0
        self.avg_processing_time = 0.0
        self.memory_usage = 0.0
        self.cpu_usage = 0.0
        self.gpu_usage = 0.0
        self.temperature = 0.0
        
        # Health status
        self.health_status = "OK"
        self.health_warnings = []
        
        # Threading for system monitoring
        self.monitoring_thread = threading.Thread(target=self.system_monitoring_loop, daemon=True)
        self.monitoring_thread.start()
        
        # Performance visualization
        self.performance_image = None
        self.create_performance_display()
        
        rospy.loginfo("Performance Monitor Node initialized")
        rospy.loginfo("Target FPS: %.1f", self.target_fps)
        rospy.loginfo("Max processing time: %.3fs", self.max_processing_time)
    
    def create_performance_display(self):
        """Create performance visualization display"""
        # Create a blank image for performance display
        self.performance_image = np.zeros((600, 800, 3), dtype=np.uint8)
    
    def filtering_stats_callback(self, stats_msg):
        """Callback for filtering statistics"""
        if len(stats_msg.data) >= 10:
            stats = {
                'timestamp': time.time(),
                'original_points': stats_msg.data[0],
                'roi_points': stats_msg.data[1],
                'voxel_points': stats_msg.data[2],
                'ground_points': stats_msg.data[3],
                'obstacle_points': stats_msg.data[4],
                'radius_points': stats_msg.data[5],
                'statistical_points': stats_msg.data[6],
                'cluster_points': stats_msg.data[7],
                'temporal_points': stats_msg.data[8],
                'processing_time': stats_msg.data[9]
            }
            
            self.filtering_stats.append(stats)
            self.update_performance_metrics()
    
    def detection_stats_callback(self, stats_msg):
        """Callback for detection statistics"""
        if len(stats_msg.data) >= 3:
            stats = {
                'timestamp': time.time(),
                'detections': stats_msg.data[0],
                'tracks': stats_msg.data[1],
                'processing_time': stats_msg.data[2]
            }
            
            self.detection_stats.append(stats)
    
    def calibration_quality_callback(self, quality_msg):
        """Callback for calibration quality metrics"""
        if len(quality_msg.data) >= 4:
            stats = {
                'timestamp': time.time(),
                'targets': quality_msg.data[0],
                'min_targets': quality_msg.data[1],
                'quality_score': quality_msg.data[2],
                'remaining_time': quality_msg.data[3]
            }
            
            self.calibration_stats.append(stats)
    
    def system_monitoring_loop(self):
        """Background thread for system monitoring"""
        while not rospy.is_shutdown():
            try:
                # Get system statistics
                self.update_system_stats()
                
                # Check system health
                self.check_system_health()
                
                # Update performance display
                self.update_performance_display()
                
                # Publish system stats
                self.publish_system_stats()
                
                # Sleep for monitoring interval
                time.sleep(1.0)
                
            except Exception as e:
                rospy.logerr("Error in system monitoring: %s", str(e))
    
    def update_system_stats(self):
        """Update system statistics"""
        # CPU usage
        self.cpu_usage = psutil.cpu_percent(interval=1)
        
        # Memory usage
        memory = psutil.virtual_memory()
        self.memory_usage = memory.percent
        
        # GPU usage (if available)
        self.gpu_usage = self.get_gpu_usage()
        
        # Temperature (if available)
        self.temperature = self.get_temperature()
        
        # Store stats
        stats = {
            'timestamp': time.time(),
            'cpu_usage': self.cpu_usage,
            'memory_usage': self.memory_usage,
            'gpu_usage': self.gpu_usage,
            'temperature': self.temperature
        }
        
        self.system_stats.append(stats)
    
    def get_gpu_usage(self):
        """Get GPU usage percentage"""
        try:
            # Try to get GPU usage using nvidia-smi
            result = subprocess.run(['nvidia-smi', '--query-gpu=utilization.gpu', '--format=csv,noheader,nounits'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                return float(result.stdout.strip())
        except:
            pass
        
        return 0.0
    
    def get_temperature(self):
        """Get system temperature"""
        try:
            # Try to get CPU temperature
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp = float(f.read()) / 1000.0
                return temp
        except:
            pass
        
        return 0.0
    
    def check_system_health(self):
        """Check system health and generate warnings"""
        self.health_warnings = []
        
        # Check CPU usage
        if self.cpu_usage > self.max_cpu_usage:
            self.health_warnings.append(f"High CPU usage: {self.cpu_usage:.1f}%")
        
        # Check memory usage
        if self.memory_usage > self.max_memory_usage:
            self.health_warnings.append(f"High memory usage: {self.memory_usage:.1f}%")
        
        # Check processing time
        if self.avg_processing_time > self.max_processing_time:
            self.health_warnings.append(f"Slow processing: {self.avg_processing_time:.3f}s")
        
        # Check FPS
        if self.current_fps < self.target_fps * 0.8:
            self.health_warnings.append(f"Low FPS: {self.current_fps:.1f}")
        
        # Check temperature
        if self.temperature > 80.0:
            self.health_warnings.append(f"High temperature: {self.temperature:.1f}°C")
        
        # Update health status
        if len(self.health_warnings) == 0:
            self.health_status = "OK"
        elif len(self.health_warnings) <= 2:
            self.health_status = "WARNING"
        else:
            self.health_status = "CRITICAL"
    
    def update_performance_metrics(self):
        """Update performance metrics from collected statistics"""
        if len(self.filtering_stats) > 0:
            # Calculate average processing time
            processing_times = [stats['processing_time'] for stats in self.filtering_stats]
            self.avg_processing_time = np.mean(processing_times)
            
            # Calculate FPS
            if len(processing_times) > 1:
                time_diffs = np.diff([stats['timestamp'] for stats in self.filtering_stats])
                self.current_fps = 1.0 / np.mean(time_diffs)
    
    def update_performance_display(self):
        """Update performance visualization display"""
        if self.performance_image is None:
            self.create_performance_display()
        
        # Clear image
        self.performance_image.fill(0)
        
        # Draw performance metrics
        y_offset = 30
        line_height = 25
        
        # System status
        status_color = (0, 255, 0) if self.health_status == "OK" else (0, 255, 255) if self.health_status == "WARNING" else (0, 0, 255)
        cv2.putText(self.performance_image, f"System Status: {self.health_status}", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        y_offset += line_height * 2
        
        # Performance metrics
        cv2.putText(self.performance_image, f"FPS: {self.current_fps:.1f} (Target: {self.target_fps})", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_offset += line_height
        
        cv2.putText(self.performance_image, f"Processing Time: {self.avg_processing_time:.3f}s", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_offset += line_height
        
        # System resources
        cv2.putText(self.performance_image, f"CPU: {self.cpu_usage:.1f}%", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_offset += line_height
        
        cv2.putText(self.performance_image, f"Memory: {self.memory_usage:.1f}%", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_offset += line_height
        
        cv2.putText(self.performance_image, f"GPU: {self.gpu_usage:.1f}%", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_offset += line_height
        
        cv2.putText(self.performance_image, f"Temperature: {self.temperature:.1f}°C", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_offset += line_height * 2
        
        # Filtering statistics
        if len(self.filtering_stats) > 0:
            latest_stats = self.filtering_stats[-1]
            cv2.putText(self.performance_image, "Filtering Statistics:", 
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            y_offset += line_height
            
            cv2.putText(self.performance_image, f"Original: {latest_stats['original_points']:.0f}", 
                       (20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_offset += line_height
            
            cv2.putText(self.performance_image, f"Filtered: {latest_stats['temporal_points']:.0f}", 
                       (20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_offset += line_height
            
            reduction = (1 - latest_stats['temporal_points'] / latest_stats['original_points']) * 100 if latest_stats['original_points'] > 0 else 0
            cv2.putText(self.performance_image, f"Reduction: {reduction:.1f}%", 
                       (20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_offset += line_height * 2
        
        # Detection statistics
        if len(self.detection_stats) > 0:
            latest_detection = self.detection_stats[-1]
            cv2.putText(self.performance_image, "Detection Statistics:", 
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            y_offset += line_height
            
            cv2.putText(self.performance_image, f"Detections: {latest_detection['detections']:.0f}", 
                       (20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_offset += line_height
            
            cv2.putText(self.performance_image, f"Active Tracks: {latest_detection['tracks']:.0f}", 
                       (20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_offset += line_height * 2
        
        # Health warnings
        if len(self.health_warnings) > 0:
            cv2.putText(self.performance_image, "Health Warnings:", 
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            y_offset += line_height
            
            for warning in self.health_warnings[:3]:  # Show first 3 warnings
                cv2.putText(self.performance_image, f"- {warning}", 
                           (20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                y_offset += line_height
        
        # Draw performance graphs
        self.draw_performance_graphs()
        
        # Publish performance image
        try:
            performance_msg = self.bridge.cv2_to_imgmsg(self.performance_image, "bgr8")
            performance_msg.header.stamp = rospy.Time.now()
            self.performance_image_pub.publish(performance_msg)
        except Exception as e:
            rospy.logwarn("Failed to publish performance image: %s", str(e))
    
    def draw_performance_graphs(self):
        """Draw performance graphs on the display"""
        if len(self.system_stats) < 2:
            return
        
        # CPU usage graph
        graph_x = 400
        graph_y = 50
        graph_width = 350
        graph_height = 100
        
        # Draw graph background
        cv2.rectangle(self.performance_image, (graph_x, graph_y), 
                     (graph_x + graph_width, graph_y + graph_height), (50, 50, 50), -1)
        cv2.rectangle(self.performance_image, (graph_x, graph_y), 
                     (graph_x + graph_width, graph_y + graph_height), (255, 255, 255), 1)
        
        # Draw CPU usage line
        cpu_values = [stats['cpu_usage'] for stats in self.system_stats[-50:]]  # Last 50 samples
        if len(cpu_values) > 1:
            points = []
            for i, value in enumerate(cpu_values):
                x = graph_x + int(i * graph_width / (len(cpu_values) - 1))
                y = graph_y + graph_height - int(value * graph_height / 100)
                points.append((x, y))
            
            if len(points) > 1:
                for i in range(len(points) - 1):
                    cv2.line(self.performance_image, points[i], points[i + 1], (0, 255, 0), 2)
        
        # Draw threshold line
        threshold_y = graph_y + graph_height - int(self.max_cpu_usage * graph_height / 100)
        cv2.line(self.performance_image, (graph_x, threshold_y), 
                (graph_x + graph_width, threshold_y), (0, 0, 255), 1)
        
        # Add labels
        cv2.putText(self.performance_image, "CPU Usage", (graph_x, graph_y - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(self.performance_image, "100%", (graph_x - 30, graph_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(self.performance_image, "0%", (graph_x - 30, graph_y + graph_height), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    
    def publish_system_stats(self):
        """Publish system statistics"""
        stats_msg = Float32MultiArray()
        stats_msg.data = [
            self.current_fps,
            self.avg_processing_time,
            self.cpu_usage,
            self.memory_usage,
            self.gpu_usage,
            self.temperature,
            float(len(self.health_warnings))
        ]
        self.system_stats_pub.publish(stats_msg)
        
        # Publish health status
        health_msg = String()
        health_msg.data = f"{self.health_status}: {', '.join(self.health_warnings)}"
        self.health_status_pub.publish(health_msg)
    
    def generate_parameter_suggestions(self):
        """Generate parameter optimization suggestions"""
        suggestions = []
        
        # Check if processing is too slow
        if self.avg_processing_time > self.max_processing_time:
            suggestions.append("Increase voxel_leaf_size to reduce processing time")
            suggestions.append("Reduce temporal_window_size for faster processing")
            suggestions.append("Increase cluster_eps to reduce clustering complexity")
        
        # Check if memory usage is high
        if self.memory_usage > self.max_memory_usage:
            suggestions.append("Reduce max_targets in calibration")
            suggestions.append("Decrease temporal_window_size")
            suggestions.append("Increase voxel_leaf_size for memory efficiency")
        
        # Check if CPU usage is high
        if self.cpu_usage > self.max_cpu_usage:
            suggestions.append("Reduce statistical_outlier_k")
            suggestions.append("Increase radius_outlier_radius")
            suggestions.append("Disable temporal consistency filtering")
        
        # Check if FPS is too low
        if self.current_fps < self.target_fps * 0.8:
            suggestions.append("Skip frames when processing is slow")
            suggestions.append("Reduce detection confidence threshold")
            suggestions.append("Use simpler filtering algorithms")
        
        return suggestions
    
    def save_performance_log(self):
        """Save performance log to file"""
        try:
            log_data = {
                'timestamp': datetime.now().isoformat(),
                'system_stats': list(self.system_stats),
                'filtering_stats': list(self.filtering_stats),
                'detection_stats': list(self.detection_stats),
                'calibration_stats': list(self.calibration_stats),
                'health_status': self.health_status,
                'health_warnings': self.health_warnings
            }
            
            log_file = os.path.expanduser(f"~/camera_lidar_performance_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json")
            
            with open(log_file, 'w') as f:
                json.dump(log_data, f, indent=2, default=str)
            
            rospy.loginfo("Performance log saved to %s", log_file)
            
        except Exception as e:
            rospy.logerr("Failed to save performance log: %s", str(e))

if __name__ == '__main__':
    try:
        node = PerformanceMonitorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
