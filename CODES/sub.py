import time
import json
import serial
import RPi.GPIO as GPIO
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient


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
def process_command(client, userdata, message):
    try:
        #print("Received message: ", message.payload)
        payload = json.loads(message.payload)
        value_x = payload.get("linear_velocity", 0)
        value_z = payload.get("angular_velocity", 0)
        distance = payload.get("distance", 0)

        print(f"Received values: value_x={value_x}, value_z={value_z}, distance={distance}")


        l_pwm = 0
        r_pwm = 0

         if value_x != 0:
            l_pwm = int((value_x * 20) + 11)
            r_pwm = int((value_x * 20) + 11)

        if value_z != 0:
            if value_z > 0:  # Turn right
                l_pwm = int(value_z + 20)
                r_pwm = int(-(value_z + 20))
            else:  # Turn left
                l_pwm = int(-(value_z + 23))
                r_pwm = int(value_z + 23)

        if l_pwm < 0:
            l_pwm *= -1
            l_pwm |= 0x80

        if r_pwm < 0:
            r_pwm *= -1
            r_pwm |= 0x80

        cmd_packet = [0xF6, 0xF7, 0x01, 0x02, 0xF0, 0x09, 0x01, 0x09, 0x08, 0x09, l_pwm, r_pwm, 0x12, 0x13, 0xF8]
        ser.write(serial.to_bytes(cmd_packet))
        print("Sent command packet including LPWM and RPWM!")

        if value_x == 0 and value_z == 0:
            stop_vehicle()
        else:
            time.sleep(1)

    except Exception as e:
        print(f"Error processing command: {e}")

if __name__ == '__main__':
    stop_vehicle()
    ser = serial.Serial(port='/dev/ttyS0', baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=2)
    #ser = serial.Serial(port='/dev/ttyAMA0', baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)

    my_mqtt_client = AWSIoTMQTTClient("Data")
    my_mqtt_client.configureEndpoint("a2yfm7zykvqeaz-ats.iot.us-east-1.amazonaws.com", 8883)
    my_mqtt_client.configureCredentials("AmazonRootCA1.pem", "private.pem.key", "certificate.pem")
    my_mqtt_client.connect()

    print("Initiating IoT Core Topic...")
    my_mqtt_client.subscribe("robot/control", 1, lambda x, y, z: process_command(x, y, z))

    while True:
        time.sleep(0.1)
