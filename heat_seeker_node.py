import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from adafruit_amg88xx import AMG88XX
import board
import busio

class HeatSeekerNode(Node):
    def __init__(self):
        super().__init__('heat_seeker')
        self.get_logger().info(" Heat Seeker Node Started")

        # AMG8833 setup
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = AMG88XX(i2c)

        # Stop and resume services (from laptop)
        self.stop_client = self.create_client(Trigger, 'stop_navigation')
        self.resume_client = self.create_client(Trigger, 'resume_navigation')

        # Wait for both services
        self.wait_for_service(self.stop_client, 'stop_navigation')
        self.wait_for_service(self.resume_client, 'resume_navigation')

        # Heat check every second
        self.timer = self.create_timer(1.0, self.check_heat)

        self.has_launched = False

    def wait_for_service(self, client, name):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for `{name}` service...')

    def check_heat(self):
        pixels = self.sensor.pixels
        max_temp = max(max(row) for row in pixels)
        self.get_logger().info(f"Heat detected! temp: {max_temp:.2f}°C")

        if max_temp > 31 and not self.has_launched:
            self.get_logger().info(f"Heat detected! Max temp: {max_temp:.2f}°C")

            # Stop navigation
            req_stop = Trigger.Request()
            self.has_launched = True  # Prevent multiple calls even if resume fails
            future_stop = self.stop_client.call_async(req_stop)
            future_stop.add_done_callback(self.handle_stop_response)

        else:
            self.heat_pub.publish(Bool(data=False))

    def handle_stop_response(self, future):
        try:
            if future.result().success:
                self.get_logger().info(" Navigation stopped. Launching...")
                self.launch_projectile()
            else:
                self.get_logger().error(" Failed to stop navigation.")
                self.has_launched = False
        except Exception as e:
            self.get_logger().error(f"Exception in stop service: {e}")
            self.has_launched = False

    def launch_projectile(self):
        import launcher  # Directly import and call launch function
        launcher.launch()
        self.get_logger().info(" Launch complete.")

        self.has_launched = True  # Prevent re-launching the same heat

        # Resume navigation
        self.get_logger().info("Sending resume command...")
        resume_req = Trigger.Request()
        self.resume_client.call_async(resume_req).add_done_callback(self.handle_resume)

    def handle_resume(self, future):
        try:
            if future.result().success:
                self.get_logger().info(" Navigation resumed.")
            else:
                self.get_logger().error(" Failed to resume navigation.")
        except Exception as e:
            self.get_logger().error(f"Exception in resume service: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HeatSeekerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
