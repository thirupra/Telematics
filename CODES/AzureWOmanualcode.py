import os
import time
import rospy
from geometry_msgs.msg import Twist
from azure.iot.device import IoTHubDeviceClient, Message

class Immobilizer:
    def __init__(self, fifo_path):
        self.fifo_path = fifo_path
        self.create_fifo()
        self.init_azure_iot()
        rospy.init_node("robot_controller_node")
        print("ROS successfully initialized")
        self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.linear_speed = 0.5
        self.angular_speed = 1.0

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

    def stop_robot(self):
        print("Stopping the robot...")
        self.move_robot(0.0, 0.0)

    def message_received_handler(self, message):
    #message_received_handler method in the Immobilizer class is triggered whenever a message is received from Azure IoT Hub. 
    #This method is set as the callback for handling incoming messages.
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
            command = self.read_from_fifo()
            #command = input("Enter command (m/s): ").strip().lower()
            if command == "s":
                self.stop_robot()

            elif command == "m":
                self.move_robot(self.linear_speed, 0.0)
                print("Moving the robot...")

            time.sleep(0.1)

if __name__ == "__main__":
    fifo_path = "/home/ubuntu/catkin_ws/src/Azure/securityservice/fifo_file"
    immobilizer = Immobilizer(fifo_path)
    immobilizer.run()
