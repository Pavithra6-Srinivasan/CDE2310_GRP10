import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import RPi.GPIO as GPIO
import time

# Motor A pins
ENA = 20
IN1 = 19
IN2 = 13

# Motor B pins
ENB = 21
IN3 = 6
IN4 = 5

#pin for solenoid
SOLENOID = 16

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Setup pins as output
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

GPIO.setup(SOLENOID, GPIO.OUT)

#pwm
pwmA = GPIO.PWM(ENA,1000)
pwmB = GPIO.PWM(ENB,1000)
pwmA.start(0)
pwmB.start(0)

def motor_forward(speed):
    # Motor A forward
    pwmA.ChangeDutyCycle(speed)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)

    # Motor B forward
    pwmB.ChangeDutyCycle(speed)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def motor_backward(speed):
    # Motor A backward
    pwmA.ChangeDutyCycle(speed)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)

    # Motor B backward
    pwmB.ChangeDutyCycle(speed)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def motor_stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwmA.ChangeDutyCycle(0)
    pwmB.ChangeDutyCycle(0)

def solenoid_activation():
    GPIO.output(SOLENOID, GPIO.HIGH)
    time.sleep(1)
    GPIO.output(SOLENOID, GPIO.LOW)
    print("launched 1 ball")

def test():
    print("a")
    GPIO.output(SOLENOID, GPIO.HIGH)
    time.sleep(10)
    GPIO.output(SOLENOID, GPIO.LOW)
    print("b")

def launch():
    motor_forward(75)
    print("Motors moving")
    #time.sleep(1)
    solenoid_activation() #1+1 delay
    time.sleep(3)
    solenoid_activation() #3+1 delay
    time.sleep(1)
    solenoid_activation() #1+1 delay
    time.sleep(1) #to give time for ball to leave
    motor_stop()
    print("Motors stopped")

# ROS2 service server node
class LauncherNode(Node):
    def __init__(self):
        super().__init__('launcher_node')
        self.srv = self.create_service(Trigger, 'launch_projectile', self.launch_cb)
        self.get_logger().info('Launcher service ready.')

    def launch_cb(self, request, response):
        self.get_logger().info(" Launch triggered via service.")
        launch()
        response.success = True
        response.message = "Projectile launched"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LauncherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
