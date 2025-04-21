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
SOLENOID = 11

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

# launching ping pong while stationary
def test():
    motor_forward(62)
    print("Motors moving")
    time.sleep(1)
    solenoid_activation() #1+1 delay
    motor_forward(52)
    time.sleep(3)
    solenoid_activation() #3+1 delay
    motor_forward(52)
    time.sleep(1)
    solenoid_activation() #1+1 delay
    time.sleep(1)
    motor_stop()
    print("Motors stopped")

# launching ping pong while navigating
def launch():
    motor_forward(70)
    print("Motors moving")
    time.sleep(1)
    solenoid_activation() #1+1 delay
    motor_forward(60)
    time.sleep(3)
    solenoid_activation() #3+1 delay
    motor_forward(60)
    time.sleep(1)
    solenoid_activation() #1+1 delay
    time.sleep(1) 
    motor_stop()
    print("Motors stopped")

if __name__ == '__main__':
    main()
def on():
        GPIO.output(SOLENOID, GPIO.HIGH)
        print("ON")
def off():
        GPIO.output(SOLENOID,GPIO.LOW)
        print("OFF")
