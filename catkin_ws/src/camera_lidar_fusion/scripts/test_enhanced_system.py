#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Test Script for Enhanced Camera-LiDAR Fusion System
Verifies that all components are working correctly on Jetson Nano.
"""

import rospy
import time
import subprocess
import os
from std_msgs.msg import String, Float32MultiArray

class EnhancedSystemTester:
    def __init__(self):
        rospy.init_node('enhanced_system_tester', anonymous=True)
        
        # Test results
        self.test_results = {}
        self.test_completed = False
        
        # Subscribers for monitoring
        self.filtering_stats_sub = rospy.Subscriber('/fused/filtering_stats', 
                                                   Float32MultiArray, self.filtering_stats_callback)
        self.system_stats_sub = rospy.Subscriber('/monitor/system_stats', 
                                                Float32MultiArray, self.system_stats_callback)
        self.health_status_sub = rospy.Subscriber('/monitor/health_status', 
                                                 String, self.health_status_callback)
        
        # Test state
        self.start_time = time.time()
        self.test_duration = 30  # seconds
        self.filtering_received = False
        self.system_stats_received = False
        self.health_status_received = False
        
        rospy.loginfo("Enhanced System Tester initialized")
        rospy.loginfo("Test duration: %d seconds", self.test_duration)
    
    def filtering_stats_callback(self, msg):
        """Callback for filtering statistics"""
        if len(msg.data) >= 10:
            self.test_results['filtering'] = {
                'original_points': msg.data[0],
                'filtered_points': msg.data[8],
                'processing_time': msg.data[9],
                'reduction_percentage': (1 - msg.data[8] / msg.data[0]) * 100 if msg.data[0] > 0 else 0
            }
            self.filtering_received = True
            rospy.loginfo("Filtering stats received: %d -> %d points (%.1f%% reduction)", 
                         msg.data[0], msg.data[8], self.test_results['filtering']['reduction_percentage'])
    
    def system_stats_callback(self, msg):
        """Callback for system statistics"""
        if len(msg.data) >= 7:
            self.test_results['system'] = {
                'fps': msg.data[0],
                'processing_time': msg.data[1],
                'cpu_usage': msg.data[2],
                'memory_usage': msg.data[3],
                'gpu_usage': msg.data[4],
                'temperature': msg.data[5],
                'health_warnings': msg.data[6]
            }
            self.system_stats_received = True
            rospy.loginfo("System stats received: FPS=%.1f, CPU=%.1f%%, Memory=%.1f%%", 
                         msg.data[0], msg.data[2], msg.data[3])
    
    def health_status_callback(self, msg):
        """Callback for health status"""
        self.test_results['health'] = msg.data
        self.health_status_received = True
        rospy.loginfo("Health status received: %s", msg.data)
    
    def check_hardware_connections(self):
        """Check hardware connections"""
        rospy.loginfo("Checking hardware connections...")
        
        # Check ZED camera
        if os.path.exists('/dev/video0'):
            rospy.loginfo("✓ ZED camera detected")
            self.test_results['hardware'] = {'zed_camera': True}
        else:
            rospy.logwarn("✗ ZED camera not found")
            self.test_results['hardware'] = {'zed_camera': False}
        
        # Check LiDAR
        if os.path.exists('/dev/ttyUSB0'):
            rospy.loginfo("✓ LiDAR detected on /dev/ttyUSB0")
            self.test_results['hardware']['lidar'] = True
        elif os.path.exists('/dev/ttyUSB1'):
            rospy.loginfo("✓ LiDAR detected on /dev/ttyUSB1")
            self.test_results['hardware']['lidar'] = True
        else:
            rospy.logwarn("✗ LiDAR not found")
            self.test_results['hardware']['lidar'] = False
    
    def check_system_resources(self):
        """Check system resources"""
        rospy.loginfo("Checking system resources...")
        
        try:
            # Check memory
            result = subprocess.run(['free', '-m'], capture_output=True, text=True)
            if result.returncode == 0:
                lines = result.stdout.split('\n')
                mem_line = lines[1].split()
                total_mem = int(mem_line[1])
                free_mem = int(mem_line[6])
                mem_usage = ((total_mem - free_mem) / total_mem) * 100
                
                rospy.loginfo("Memory: %dMB total, %dMB free (%.1f%% used)", 
                             total_mem, free_mem, mem_usage)
                self.test_results['resources'] = {
                    'total_memory_mb': total_mem,
                    'free_memory_mb': free_mem,
                    'memory_usage_percent': mem_usage
                }
            
            # Check CPU
            result = subprocess.run(['nproc'], capture_output=True, text=True)
            if result.returncode == 0:
                cpu_cores = int(result.stdout.strip())
                rospy.loginfo("CPU: %d cores", cpu_cores)
                self.test_results['resources']['cpu_cores'] = cpu_cores
            
            # Check GPU (if available)
            result = subprocess.run(['nvidia-smi', '--query-gpu=memory.total', 
                                   '--format=csv,noheader,nounits'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                gpu_mem = int(result.stdout.strip())
                rospy.loginfo("GPU: %dMB total memory", gpu_mem)
                self.test_results['resources']['gpu_memory_mb'] = gpu_mem
            else:
                rospy.loginfo("GPU: Not available or nvidia-smi not found")
                self.test_results['resources']['gpu_memory_mb'] = 0
                
        except Exception as e:
            rospy.logwarn("Error checking system resources: %s", str(e))
    
    def check_ros_topics(self):
        """Check if required ROS topics are available"""
        rospy.loginfo("Checking ROS topics...")
        
        required_topics = [
            '/zed/left/image_raw',
            '/unilidar/cloud',
            '/fused/enhanced_cloud',
            '/monitor/system_stats',
            '/monitor/health_status'
        ]
        
        available_topics = []
        for topic in required_topics:
            try:
                # Try to get topic info
                topic_info = rospy.get_published_topics()
                topic_names = [info[0] for info in topic_info]
                
                if topic in topic_names:
                    available_topics.append(topic)
                    rospy.loginfo("✓ Topic available: %s", topic)
                else:
                    rospy.logwarn("✗ Topic not available: %s", topic)
            except:
                rospy.logwarn("✗ Topic not available: %s", topic)
        
        self.test_results['ros_topics'] = {
            'required': required_topics,
            'available': available_topics,
            'coverage': len(available_topics) / len(required_topics) * 100
        }
        
        rospy.loginfo("ROS topic coverage: %.1f%%", self.test_results['ros_topics']['coverage'])
    
    def run_performance_test(self):
        """Run performance test for specified duration"""
        rospy.loginfo("Running performance test for %d seconds...", self.test_duration)
        
        start_time = time.time()
        while not rospy.is_shutdown() and (time.time() - start_time) < self.test_duration:
            time.sleep(1)
            
            # Log progress
            elapsed = time.time() - start_time
            if int(elapsed) % 5 == 0:
                rospy.loginfo("Test progress: %.1f seconds elapsed", elapsed)
        
        rospy.loginfo("Performance test completed")
    
    def evaluate_test_results(self):
        """Evaluate test results and generate report"""
        rospy.loginfo("Evaluating test results...")
        
        # Check if all required data was received
        if not self.filtering_received:
            rospy.logwarn("✗ No filtering statistics received")
        
        if not self.system_stats_received:
            rospy.logwarn("✗ No system statistics received")
        
        if not self.health_status_received:
            rospy.logwarn("✗ No health status received")
        
        # Evaluate performance
        performance_score = 0
        max_score = 100
        
        if 'system' in self.test_results:
            system = self.test_results['system']
            
            # FPS score (target: 3+ Hz)
            if system['fps'] >= 3.0:
                performance_score += 25
                rospy.loginfo("✓ FPS: %.1f Hz (target: 3.0+)", system['fps'])
            else:
                rospy.logwarn("✗ FPS: %.1f Hz (target: 3.0+)", system['fps'])
            
            # CPU usage score (target: <85%)
            if system['cpu_usage'] < 85.0:
                performance_score += 25
                rospy.loginfo("✓ CPU usage: %.1f%% (target: <85%%)", system['cpu_usage'])
            else:
                rospy.logwarn("✗ CPU usage: %.1f%% (target: <85%%)", system['cpu_usage'])
            
            # Memory usage score (target: <75%)
            if system['memory_usage'] < 75.0:
                performance_score += 25
                rospy.loginfo("✓ Memory usage: %.1f%% (target: <75%%)", system['memory_usage'])
            else:
                rospy.logwarn("✗ Memory usage: %.1f%% (target: <75%%)", system['memory_usage'])
            
            # Processing time score (target: <300ms)
            if system['processing_time'] < 0.3:
                performance_score += 25
                rospy.loginfo("✓ Processing time: %.3fs (target: <0.3s)", system['processing_time'])
            else:
                rospy.logwarn("✗ Processing time: %.3fs (target: <0.3s)", system['processing_time'])
        
        # Evaluate filtering
        if 'filtering' in self.test_results:
            filtering = self.test_results['filtering']
            if filtering['reduction_percentage'] > 50:
                rospy.loginfo("✓ Point reduction: %.1f%% (target: >50%%)", filtering['reduction_percentage'])
            else:
                rospy.logwarn("✗ Point reduction: %.1f%% (target: >50%%)", filtering['reduction_percentage'])
        
        # Evaluate health
        if 'health' in self.test_results:
            health = self.test_results['health']
            if 'OK' in health:
                rospy.loginfo("✓ Health status: %s", health)
            else:
                rospy.logwarn("✗ Health status: %s", health)
        
        # Overall evaluation
        self.test_results['performance_score'] = performance_score
        self.test_results['max_score'] = max_score
        
        if performance_score >= 75:
            rospy.loginfo("🎉 Test PASSED! Performance score: %d/%d", performance_score, max_score)
            self.test_results['overall_result'] = 'PASS'
        elif performance_score >= 50:
            rospy.logwarn("⚠️  Test PARTIAL! Performance score: %d/%d", performance_score, max_score)
            self.test_results['overall_result'] = 'PARTIAL'
        else:
            rospy.logerr("❌ Test FAILED! Performance score: %d/%d", performance_score, max_score)
            self.test_results['overall_result'] = 'FAIL'
    
    def print_test_report(self):
        """Print comprehensive test report"""
        print("\n" + "="*60)
        print("ENHANCED CAMERA-LIDAR FUSION SYSTEM TEST REPORT")
        print("="*60)
        
        print(f"\nTest Duration: {self.test_duration} seconds")
        print(f"Overall Result: {self.test_results.get('overall_result', 'UNKNOWN')}")
        print(f"Performance Score: {self.test_results.get('performance_score', 0)}/{self.test_results.get('max_score', 100)}")
        
        # Hardware status
        if 'hardware' in self.test_results:
            print(f"\nHardware Status:")
            for device, status in self.test_results['hardware'].items():
                status_symbol = "✓" if status else "✗"
                print(f"  {status_symbol} {device.replace('_', ' ').title()}")
        
        # System resources
        if 'resources' in self.test_results:
            res = self.test_results['resources']
            print(f"\nSystem Resources:")
            print(f"  Memory: {res.get('total_memory_mb', 0)}MB total, {res.get('free_memory_mb', 0)}MB free")
            print(f"  CPU: {res.get('cpu_cores', 0)} cores")
            print(f"  GPU: {res.get('gpu_memory_mb', 0)}MB")
        
        # Performance metrics
        if 'system' in self.test_results:
            sys = self.test_results['system']
            print(f"\nPerformance Metrics:")
            print(f"  FPS: {sys.get('fps', 0):.1f} Hz")
            print(f"  Processing Time: {sys.get('processing_time', 0):.3f}s")
            print(f"  CPU Usage: {sys.get('cpu_usage', 0):.1f}%")
            print(f"  Memory Usage: {sys.get('memory_usage', 0):.1f}%")
            print(f"  GPU Usage: {sys.get('gpu_usage', 0):.1f}%")
            print(f"  Temperature: {sys.get('temperature', 0):.1f}°C")
        
        # Filtering performance
        if 'filtering' in self.test_results:
            filt = self.test_results['filtering']
            print(f"\nFiltering Performance:")
            print(f"  Original Points: {filt.get('original_points', 0):.0f}")
            print(f"  Filtered Points: {filt.get('filtered_points', 0):.0f}")
            print(f"  Reduction: {filt.get('reduction_percentage', 0):.1f}%")
        
        # Health status
        if 'health' in self.test_results:
            print(f"\nHealth Status:")
            print(f"  {self.test_results['health']}")
        
        # ROS topics
        if 'ros_topics' in self.test_results:
            topics = self.test_results['ros_topics']
            print(f"\nROS Topics:")
            print(f"  Coverage: {topics.get('coverage', 0):.1f}%")
            print(f"  Available: {len(topics.get('available', []))}/{len(topics.get('required', []))}")
        
        print("\n" + "="*60)
        print("Test completed!")
        print("="*60)
    
    def run_tests(self):
        """Run all tests"""
        rospy.loginfo("Starting Enhanced System Tests...")
        
        # Wait for system to initialize
        rospy.sleep(5)
        
        # Run hardware checks
        self.check_hardware_connections()
        self.check_system_resources()
        
        # Wait a bit more for ROS topics to appear
        rospy.sleep(3)
        
        # Check ROS topics
        self.check_ros_topics()
        
        # Run performance test
        self.run_performance_test()
        
        # Evaluate results
        self.evaluate_test_results()
        
        # Print report
        self.print_test_report()
        
        self.test_completed = True

if __name__ == '__main__':
    try:
        tester = EnhancedSystemTester()
        tester.run_tests()
    except rospy.ROSInterruptException:
        pass
