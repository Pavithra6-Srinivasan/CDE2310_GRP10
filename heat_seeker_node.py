import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from adafruit_amg88xx import AMG88XX
import board
import busio
import time
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class HeatSeekerNode(Node):
    def __init__(self):
        super().__init__('heat_seeker')
        self.get_logger().info(" Heat Seeker Node Started")

        # AMG8833 setup
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            while not i2c.try_lock():
                time.sleep(0.1)
            i2c.unlock()
            self.sensor = AMG88XX(i2c)
            self.get_logger().info("AMG88XX sensor initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize sensor: {e}")
            raise

        # Create service FIRST so explorer.py can find it!
        self.create_service(Trigger, 'trigger_launcher', self.trigger_launcher_callback)
        self.get_logger().info("trigger_launcher service created.")

        # Stop and resume services (from laptop)
        self.stop_client = self.create_client(Trigger, 'stop_navigation')
        self.resume_client = self.create_client(Trigger, 'resume_navigation')

        # Heat detection parameters
        self.heat_threshold = 30.0
        self.heat_detection_count = 0
        self.detection_threshold = 1  # Require 3 consecutive detections
        self.has_launched = False

        # Publisher to notify explorer
        self.heat_pub = self.create_publisher(Bool, '/heat_detected', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Faster checking rate
        self.timer = self.create_timer(0.5, self.check_heat)

    def check_heat(self):
        try:
            pixels = self.sensor.pixels
            max_temp = np.max(pixels)

            # Print the 8x8 matrix
            self.get_logger().info("AMG8833 Temperature Grid (°C):")
            for row in pixels:
                row_str = " ".join(f"{temp:5.1f}" for temp in row)
                self.get_logger().info(row_str)

            # Log the highest temp
            self.get_logger().info(f"Max Temperature: {max_temp:.2f}°C")

            # Heat detection logic
            if max_temp > self.heat_threshold:

                if not self.has_launched:
                    self.handle_heat_detected()
            else:
                self.heat_detection_count = 0

        except Exception as e:
            self.get_logger().error(f"Sensor error: {e}")

    def handle_heat_detected(self):
        self.get_logger().warning(" HEAT SOURCE DETECTED")
        self.has_launched = True

        # Notify explorer
        msg = Bool()
        msg.data = True
        self.heat_pub.publish(msg)

        # Stop navigation
        self.call_service(self.stop_client, "stop_navigation", self.handle_stop_response)

    def handle_stop_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Navigation stopped. Launching...")
                self.launch_projectile()
            else:
                self.get_logger().error("Failed to stop navigation")
                self.reset_launch_state()
        except Exception as e:
            self.get_logger().error(f"Stop service error: {e}")
            self.reset_launch_state()

    def launch_projectile(self):
        try:
            from . import launcher  # Local module for actual launching

            # Force stop robot immediately
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

            self.get_logger().info("=== LAUNCHING PROJECTILE ===")
            launcher.launch()
            self.get_logger().info("Launch complete!")

            # Resume navigation
            self.call_service(self.resume_client, "resume_navigation", self.handle_resume_response)
        except Exception as e:
            self.get_logger().error(f"Launch failed: {e}")
            self.reset_launch_state()

    def handle_resume_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Navigation resumed successfully")
            else:
                self.get_logger().error("Failed to resume navigation")
        except Exception as e:
            self.get_logger().error(f"Resume service error: {e}")
        finally:
            self.reset_launch_state()

    def call_service(self, client, name, callback):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f"{name} service not available")
            return False

        req = Trigger.Request()
        future = client.call_async(req)
        future.add_done_callback(callback)
        return True

    def reset_launch_state(self):
        self.has_launched = False
        self.heat_detection_count = 0

    def trigger_launcher_callback(self, request, response):
        if not self.has_launched:
            self.get_logger().info("Received trigger_launcher service call.")
            self.has_launched = True
            self.call_service(self.stop_client, "stop_navigation", self.handle_stop_response)
            response.success = True
            response.message = "Launcher triggered."
        else:
            response.success = False
            response.message = "Launcher already triggered."
        return response
def main(args=None):
    rclpy.init(args=args)
    try:
        node = HeatSeekerNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Node crashed: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
