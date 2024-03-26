#This code establishes communication between the Azure IoT cloud and the immobilizer by using FIFOs (First In, First Out) for inter-process communication.

import os
import time
import rospy
from geometry_msgs.msg import Twist
import json
import serial
import RPi.GPIO as GPIO
from azure.iot.device import IoTHubDeviceClient, Message

left = 12  # PWM pin connected to the left motor
leftDir = 5
right = 13  # PWM pin connected to the right motor
rightDir = 6

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(left, GPIO.OUT)
GPIO.setup(leftDir, GPIO.OUT)
GPIO.setup(right, GPIO.OUT)
GPIO.setup(rightDir, GPIO.OUT)

pwmLeft = GPIO.PWM(left, 1000)
pwmRight = GPIO.PWM(right, 1000)
pwmLeft.start(0)
pwmRight.start(0)

def stop_vehicle():
    print("Stopping the vehicle")
    pwmLeft.ChangeDutyCycle(0)
    pwmRight.ChangeDutyCycle(0)

class Immobilizer:
    def __init__(self, fifo_path):
        self.fifo_path = fifo_path
        self.create_fifo()
        self.init_ros()
        self.init_azure_iot()
        self.linear_speed = 0.5
        self.angular_speed = 1.0
	 def init_ros(self):
        rospy.init_node("robot_controller_node")
        print("ROS successfully initialized")
        self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def init_azure_iot(self):
        connection_str = "HostName=IoThub7.azure-devices.net;DeviceId=IoTRobotdevice2;SharedAccessKey=GrcSn2sHhJvIhDqWguWE+tZnouEexaKwOAIoTM2DmCs="
        #connection_str = "HostName=iothub4.azure-devices.net;DeviceId=IoTRobotDevice1;SharedAccessKeyName=SendCommandsPolicy;SharedAccessKey=hwLNqiJ1/D2ayEiPt9EGYIZiH4hTsQJ0GAIoTJb1vZo="
        self.device_client = IoTHubDeviceClient.create_from_connection_string(connection_str)
        self.device_client.on_message_received = self.message_received_handler

    def create_fifo(self):
        if not os.path.exists(self.fifo_path):
            os.mkfifo(self.fifo_path)

    def write_to_fifo(self, command):
        print('Write')
        with open(self.fifo_path, "w") as fifo:
            fifo.write(command)

    def read_from_fifo(self):
        print("read")
        with open(self.fifo_path, "r") as fifo:
            return fifo.read()

    def move_robot(self, linear_velocity, angular_velocity):
        cmd = Twist()
        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity
        self.publisher.publish(cmd)

        l_pwm = 0
        r_pwm = 0

        if linear_velocity != 0:
            l_pwm = int((linear_velocity * 20) + 10)
            r_pwm = int((linear_velocity * 20) + 10)
            
        elif linear_velocity < 0:
            print("Entered reverse condition...")
            l_pwm = int((linear_velocity * 10) - 10)
            r_pwm = int((linear_velocity * 10) - 10)

		if angular_velocity != 0:
            if angular_velocity > 0:  # Turn right
                l_pwm = int(angular_velocity + 18)
                r_pwm = int(-(angular_velocity + 18))
            else:  # Turn left
                l_pwm = int(-(angular_velocity + 19.5))
                r_pwm = int(angular_velocity + 19.5)

        if l_pwm < 0:
            l_pwm *= -1
            l_pwm |= 0x80

        if r_pwm < 0:
            r_pwm *= -1
            r_pwm |= 0x80

        cmd_packet = [0xF6, 0xF7, 0x01, 0x02, 0xF0, 0x09, 0x01, 0x09, 0x08, 0x09, l_pwm, r_pwm, 0x12, 0x13, 0xF8]
        ser.write(serial.to_bytes(cmd_packet))
        print("Sent command packet including LPWM={l_pwm} and RPWM={r_pwm}!")

    def stop_robot(self):
        print("Stopping the robot...")
        self.move_robot(0.0, 0.0)
        stop_vehicle()
    def message_received_handler(self, message):
        data = message.data.decode('utf-8')  # Decode bytes to string
        print("Message received:", data)
        #print(f"Received message: {message.data}")
        if data == "s":
            print('Writing S')
            self.write_to_fifo("s")

        elif data == "m":
            print('Writing M')
            self.write_to_fifo("m")
	def run(self):
        while True:
            #command = input("Enter command (m/s): ").strip().lower()
            command = self.read_from_fifo()
            if command == "s":
                self.stop_robot()
            elif command == "m":
                self.move_robot(self.linear_speed, 0.0)
                print("Moving the robot...")
            time.sleep(0.1)

if __name__ == "__main__":
    fifo_path = "/home/ubuntu/catkin_ws/src/Azure/securityservice/fifo_file"
    immobilizer = Immobilizer(fifo_path)
    ser = serial.Serial(port='/dev/ttyS0', baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)
    immobilizer.run()
