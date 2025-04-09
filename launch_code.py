import RPi.GPIO as GPIO
import time

# Motor A pins
ENA = 18  #PWM pin for Motor A **NOT CONNECTED**
IN1 = 17  
IN2 = 27

# Motor B pins
ENB = 19  # PWM pin for Motor B **NOT CONNECTED**
IN3 = 23
IN4 = 24

#pin for solenoid
SOLENOID = 21 # **NOT CONNECTED**

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

# Set up PWM for both motors
pwmA = GPIO.PWM(ENA, 1000)  # 1kHz frequency for Motor A
pwmB = GPIO.PWM(ENB, 1000)  # 1kHz frequency for Motor B

pwmA.start(0)
pwmB.start(0)

def motor_forward(speed):
    # Motor A forward
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwmA.ChangeDutyCycle(speed)

    # Motor B forward
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwmB.ChangeDutyCycle(speed)

def motor_backward(speed):
    # Motor A backward
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwmA.ChangeDutyCycle(speed)

    # Motor B backward
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwmB.ChangeDutyCycle(speed)

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

def launch():
    motor_forward(100)
    print("Motors moving")
    time.sleep(1)
    solenoid_activation() #1+1 delay
    time.sleep(3)
    solenoid_activation() #3+1 delay
    time.sleep(1)
    solenoid_activation() #1+1 delay
    time.sleep(1) #to give time for ball to leave
    motor_stop()
    print("Motors stopped")
