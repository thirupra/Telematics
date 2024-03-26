import os
import time
import rospy
from geometry_msgs.msg import Twist
import json
import serial
import RPi.GPIO as GPIO
from azure.iot.device import IoTHubDeviceClient, Message
import threading  # Import threading module
#import sys

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

BUTTON_PIN = 24
BUZZER_PIN = 16
GPIO.setmode(GPIO.BCM)

# Set up the switch pin as input with pull-up
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# Set up the buzzer pin as output
GPIO.setup(BUZZER_PIN, GPIO.OUT)

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
        self.linear_speed = 1.0
        self.angular_speed = 1.0
        self.immobilized = False  # Flag to control movement

    def init_ros(self):
        rospy.init_node("robot_controller_node")
        print("ROS successfully initialized")
        self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
	 def init_azure_iot(self):
        connection_str = "HostName=IoThub7.azure-devices.net;DeviceId=IoTRobotdevice2;SharedAccessKey=GrcSn2sHhJvIhDqWguWE+tZnouEexaKwOAIoTM2DmCs=" # Venkat
        self.device_client = IoTHubDeviceClient.create_from_connection_string(connection_str)
        self.device_client.on_message_received = self.message_received_handler

    def create_fifo(self):
        if not os.path.exists(self.fifo_path):
            os.mkfifo(self.fifo_path)

    def write_to_fifo(self, command):
        try:
            with open(self.fifo_path, "w") as fifo:
                fifo.write(command)
        except IOError as e:
            print(f"Error writing to FIFO: {e}")

    def read_from_fifo(self):
        try:
            with open(self.fifo_path, "r") as fifo:
                return fifo.read()
        except IOError as e:
            print(f"Error reading from FIFO: {e}")
            return ""

	 def move_robot(self, linear_velocity, angular_velocity):
        if self.immobilized:  # Check if immobilized
            print("The robot is immobilized. Ignoring movement command.")
            return

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

    def send_message_to_azure(self, message_body):
        message = Message(message_body)
        self.device_client.send_message(message)

    def stop_robot(self):
        print("Stopping the robot...")
        self.move_robot(0.0, 0.0)
        stop_vehicle()

    def immobilize_robot(self):
        print("Immobilizing the robot...")
        self.move_robot(0.0, 0.0)
        self.immobilized = True

    def deactivate_immobilizer(self):
        print("Deactivating the immobilizer...")
        self.immobilized = False
	def message_received_handler(self, message):
        data = message.data.decode('utf-8')  # Decode bytes to string
        print("Message received:", data)
        if data == "s":
            self.write_to_fifo("s")
        elif data == "m":
            self.write_to_fifo("m")
        elif data == 'l':
            self.write_to_fifo("l")
        elif data == 'r':
            self.write_to_fifo("r")
        elif data == 'b':
            self.write_to_fifo("b")
        elif data == "immobilize":
            print('Activating immobilizer...')
            self.immobilize_robot()
        elif data == "deactivate":
            print('Deactivating immobilizer...')
            self.deactivate_immobilizer()

    def run(self):
        while True:
            command = self.read_from_fifo()
            if command == "s":
                self.stop_robot()
            elif command == "m":
                self.move_robot(self.linear_speed, 0.0)
                print("Moving the robot...")
            elif command == 'l':
                print('Rotating left...')
                self.move_robot(self.linear_speed, -0.4)
                time.sleep(2.8)
                self.stop_robot()
            elif command == 'r':
                print('Rotating Right')
                self.move_robot(self.linear_speed, 0.4)
                time.sleep(2.8)
                self.stop_robot()
            elif command == 'b':
                self.move_robot(-1.5, 0)
                print('Moving Backward...')
            time.sleep(0.1)

def cmd_vel_callback(data):
    linear_x = data.linear.x
    angular_z = data.angular.z
    immobilizer.move_robot(linear_x, angular_z)
def switch_check():
    switch_state = GPIO.input(BUTTON_PIN)
    print("Initial Switch state:", "ON" if switch_state == GPIO.HIGH else "OFF")
    while True:
        new_switch_state = GPIO.input(BUTTON_PIN)
        if new_switch_state == GPIO.HIGH:
            print("Switch is ON")
            GPIO.output(BUZZER_PIN, GPIO.HIGH)
            print('Buzzer On')
            time.sleep(10)
            switch_state = GPIO.input(BUTTON_PIN)
            if switch_state == GPIO.HIGH:
                immobilizer.move_robot(0, 0)
                message_body1 ="Breakdown Happended"
                for _ in range(3):
                    beep()
                immobilizer.send_message_to_azure(message_body1)
                print("Breakdown Alert Sent to Azure Cloud")
                os._exit(0)
            else:
                print('Switch OFF (withdrawn)')
                GPIO.output(BUZZER_PIN, GPIO.LOW)
                print('Buzzer Off')

        switch_state = new_switch_state
    time.sleep(0.1)  # Short delay



def read_serial(ser):
    prev_ticks = [0, 0, 0, 0]
    forward_distance = 0
    stopped_counter = 0
    timer = 0
    speed = 0
    while True:
        res = ser.readline()
        try:
            split_data = res.decode('utf-8').split(',')
        except UnicodeDecodeError:
            print("Failed to decode data from serial port:", res)
            continue

        if len(split_data) >= 4:
            try:
                ticks = list(map(int, split_data))
            except ValueError:
                print("Invalid data received from serial port:", split_data)
                continue

            if ticks[0] != prev_ticks[0]:
                prev_ticks = ticks
                stopped_counter = 0
                if ticks[0] > 0:
                    forward_distance = ticks[0] / 16
                    print("Incrementing forward distance...")
                    print("Total forward distance traveled:", forward_distance, "cm")
                    timer=timer+1
                    speed = forward_distance/timer
                    speed = speed /100
                    print(f'Current Speed is:{speed:.2f} m/sec')
            else:
                stopped_counter += 1
                if stopped_counter >= 3:
                    print("Robot Vehicle is in STOP Position")
                    immobilizer.send_message_to_azure("Robot has stopped moving")
                    stopped_counter = 0
                    speed=0
                    print(f'Current Speed is:{speed:.2f} m/sec')
            message_body = f"Total forward distance: {forward_distance} cm\nCurrent Speed is:{speed:.2f} m/sec"

            immobilizer.send_message_to_azure(message_body)
    time.sleep(1)

if __name__ == "__main__":
    fifo_path = "/home/ubuntu/catkin_ws/src/Azure/securityservice/fifo_file"
    immobilizer = Immobilizer(fifo_path)
    ser = serial.Serial(port='/dev/ttyS0', baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)

    # Creating a thread for reading serial data
    serial_thread = threading.Thread(target=read_serial, args=(ser,))
    serial_thread.daemon = True
    serial_thread.start()

    # Starting the switch check thread
    switch_thread = threading.Thread(target=switch_check)
    switch_thread.daemon = True
    switch_thread.start()
	immobilizer.run()
    rospy.spin()
