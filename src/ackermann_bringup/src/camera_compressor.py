#!/usr/bin/env python3
"""
Description:
    Spawns the camera only when needed.
    Compresses the image and publishes it to
    "/camera/image/compressed".

Purpose:
    Saving Computation Power on the Pi.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Empty
from cv_bridge import CvBridge
import cv2
import subprocess
import os
import signal

class OnDemandCameraCompressor(Node):
    def __init__(self):
        super().__init__('camera_compressor_node')
        
        self.bridge = CvBridge()
        
        self.compressed_pub = self.create_publisher(
            CompressedImage,
            '/camera/image/compressed',
            10
        )
        
        self.trigger_sub = self.create_subscription(
            Empty,
            '/camera/trigger',
            self.trigger_compression_sub,
            10
        )
        
        # Only subscribe to the camera when waiting for a frame
        # This is done to save computational power on the Pi 
        self.temp_sub = None
        self.timeout_timer = None

        self.is_capturing = False
        self.camera_process = None
        
        self.get_logger().info('On-Demand Camera Trigger node initialized.')
        self.get_logger().info('Publish an Empty msg to `/camera/trigger` spawn the camera and get a frame.')
        
    def image_callback(self, msg):
        self.get_logger().info("Frame acquired! Compressing...")
        
        if self.timeout_timer:
            self.timeout_timer.cancel()
            self.timeout_timer = None
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            result, encimg = cv2.imencode('.jpg', cv_image, encode_param)
            
            if result:
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header
                compressed_msg.format = "jpeg"
                compressed_msg.data = encimg.tobytes()
                
                self.compressed_pub.publish(compressed_msg)
                self.get_logger().info('Published compressed image frame.')
            else:
                self.get_logger().error('Failed to compress image data.')
                
        except Exception as e:
            self.get_logger().error(f'Error compressing image: {str(e)}')
            
        finally:
            self._cleanup()

    def trigger_compression_sub(self, msg):
        if self.is_capturing:
            self.get_logger().warn('Already capturing an image, ignoring trigger.')
            return
            
        self.is_capturing = True
        
        self.get_logger().info("Spawning camera_ros node...")
        cmd = [
            'ros2', 'run', 'camera_ros', 'camera_node',
            '--ros-args', 
            '-r', '__node:=camera',
            '-r', '~/image_raw:=/camera/image_raw',
            '-r', 'image_raw:=/camera/image_raw',
            '-p', 'format:=RGB888',
            '-p', 'width:=1920',
            '-p', 'height:=1080',
            '-p', 'fps:=5.0'
        ]
        
        env = os.environ.copy()
        
        try:
            self.camera_process = subprocess.Popen(
                cmd, 
                stdout=subprocess.DEVNULL, 
                stderr=subprocess.DEVNULL,
                env=env,
                preexec_fn=os.setsid
            )
            
            self.temp_sub = self.create_subscription(
                Image,
                '/camera/image_raw',
                self.image_callback,
                10
            )

            self.timeout_timer = self.create_timer(15.0, self._timeout_callback)
            
        except Exception as e:
            self.get_logger().error(f"Failed to spawn camera process: {e}")
            self._cleanup()

    def _timeout_callback(self):
        self.get_logger().error("Timed out waiting for camera frame.")
        if self.timeout_timer:
            self.timeout_timer.cancel()
            self.timeout_timer = None
        self._cleanup()
            
    def _cleanup(self):
        self.get_logger().info("Shutting down camera node...")
        if self.temp_sub:
            self.destroy_subscription(self.temp_sub)
            self.temp_sub = None
            
        if self.camera_process:
            # Sometimes the camera process will not die when killing the node
            # We need to kill it manually
            try:
                os.killpg(os.getpgid(self.camera_process.pid), signal.SIGTERM)
                self.camera_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.camera_process.pid), signal.SIGKILL)
            except Exception as e:
                self.get_logger().error(f"Error killing process group: {e}")
            finally:
                self.camera_process = None
            
        self.is_capturing = False

    def destroy_node(self):
        self.get_logger().info("Compressor shutting down, ensuring camera is killed...")
        self._cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OnDemandCameraCompressor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
