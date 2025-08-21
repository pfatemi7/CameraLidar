#!/usr/bin/env python

import rospy
import json
import threading
import time
from flask import Flask, render_template_string, jsonify
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import struct

app = Flask(__name__)

# Global variables to store latest data
latest_data = {
    'lidar_status': 'No data',
    'point_count': 0,
    'timestamp': time.time(),
    'system_status': 'Running'
}

class WebMonitor:
    def __init__(self):
        rospy.init_node('web_monitor', anonymous=True)
        
        # Subscribers
        self.lidar_sub = rospy.Subscriber('/lidar/status', String, self.lidar_callback)
        self.cloud_sub = rospy.Subscriber('/fused/cloud', PointCloud2, self.cloud_callback)
        
        rospy.loginfo("Web monitor initialized")
    
    def lidar_callback(self, msg):
        global latest_data
        latest_data['lidar_status'] = msg.data
        latest_data['timestamp'] = time.time()
    
    def cloud_callback(self, msg):
        global latest_data
        latest_data['point_count'] = msg.width * msg.height
        latest_data['timestamp'] = time.time()

# HTML template for the web interface
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Camera-LiDAR Fusion Monitor</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background-color: #f0f0f0; }
        .container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
        .status-box { background: #e8f5e8; border: 1px solid #4caf50; padding: 15px; margin: 10px 0; border-radius: 5px; }
        .data-box { background: #e3f2fd; border: 1px solid #2196f3; padding: 15px; margin: 10px 0; border-radius: 5px; }
        .error-box { background: #ffebee; border: 1px solid #f44336; padding: 15px; margin: 10px 0; border-radius: 5px; }
        h1 { color: #333; text-align: center; }
        h2 { color: #666; }
        .refresh-btn { background: #4caf50; color: white; padding: 10px 20px; border: none; border-radius: 5px; cursor: pointer; }
        .refresh-btn:hover { background: #45a049; }
    </style>
    <script>
        function refreshData() {
            fetch('/api/data')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('status').innerHTML = data.lidar_status;
                    document.getElementById('points').innerHTML = data.point_count;
                    document.getElementById('timestamp').innerHTML = new Date(data.timestamp * 1000).toLocaleString();
                    document.getElementById('system').innerHTML = data.system_status;
                });
        }
        
        // Auto-refresh every 2 seconds
        setInterval(refreshData, 2000);
        
        // Initial load
        window.onload = refreshData;
    </script>
</head>
<body>
    <div class="container">
        <h1>🎯 Camera-LiDAR Fusion Monitor</h1>
        
        <div class="status-box">
            <h2>🟢 System Status</h2>
            <p><strong>Status:</strong> <span id="system">Loading...</span></p>
            <p><strong>Last Update:</strong> <span id="timestamp">Loading...</span></p>
        </div>
        
        <div class="data-box">
            <h2>📊 LiDAR Data</h2>
            <p><strong>Status:</strong> <span id="status">Loading...</span></p>
            <p><strong>Points:</strong> <span id="points">Loading...</span></p>
        </div>
        
        <div class="status-box">
            <h2>🔧 Controls</h2>
            <button class="refresh-btn" onclick="refreshData()">🔄 Refresh Data</button>
            <p><small>Auto-refreshes every 2 seconds</small></p>
        </div>
        
        <div class="data-box">
            <h2>📋 Available Topics</h2>
            <ul>
                <li><code>/unilidar/cloud</code> - Raw LiDAR data</li>
                <li><code>/fused/cloud</code> - Processed LiDAR data</li>
                <li><code>/lidar/status</code> - Processing status</li>
                <li><code>/rosout</code> - System logs</li>
            </ul>
        </div>
    </div>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/api/data')
def get_data():
    return jsonify(latest_data)

if __name__ == '__main__':
    try:
        monitor = WebMonitor()
        
        # Start Flask app in a separate thread
        def run_flask():
            app.run(host='0.0.0.0', port=8080, debug=False)
        
        flask_thread = threading.Thread(target=run_flask)
        flask_thread.daemon = True
        flask_thread.start()
        
        rospy.loginfo("Web monitor started at http://localhost:8080")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
