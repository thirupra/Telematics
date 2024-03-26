import RPi.GPIO as GPIO
import time
GPIO.setwarnings(False)
from azure.iot.device import IoTHubDeviceClient, Message

BUTTON_PIN = 24
BUZZER_PIN = 16
GPIO.setmode(GPIO.BCM)

# Set up the switch pin as input with pull-up
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# Setting up the buzzer pin as output
GPIO.setup(BUZZER_PIN, GPIO.OUT)



class AzureIoTClient:
    def __init__(self):
        self.init_azure_iot()

    def init_azure_iot(self):
        #connection_str = "HostName=FMRIoTHub1.azure-devices.net;DeviceId=RosberryPi;SharedAccessKey=3vy797qXXAanvQE+YQKOL9+d6cgLtk3fCAIoTC4ocXs=" #Subrama
        connection_str = "HostName=IoThub7.azure-devices.net;DeviceId=IoTRobotdevice2;SharedAccessKey=GrcSn2sHhJvIhDqWguWE+tZnouEexaKwOAIoTM2DmCs=" # Venkat
        self.device_client = IoTHubDeviceClient.create_from_connection_string(connection_str)
        self.device_client.on_message_received = self.message_received_handler

    def send_message_to_azure(self, message_body):
        message = Message(message_body)
        self.device_client.send_message(message)

    def message_received_handler(self, message):
        print("Message received from Azure IoT Hub:", message)


azure_client = AzureIoTClient()

try:
    while True:
        message_body = "Breakdown Happened"
        switch_state = GPIO.input(BUTTON_PIN)

        if switch_state == GPIO.LOW:
            print("Switch is OFF")
            GPIO.output(BUZZER_PIN, GPIO.LOW)  # Turning off the buzzer


        else:
            print("Switch is ON")
            GPIO.output(BUZZER_PIN, GPIO.HIGH)  # Turning on the buzzer
            print('Buzzer ON')
            time.sleep(10)
            switch_state = GPIO.input(BUTTON_PIN)
            if switch_state != GPIO.LOW:
                print("Breakdown Alert sent to Azure cloud")
                azure_client.send_message_to_azure(message_body)
                break
            else:
                print("Switch is OFF (Withdrawn)")
                GPIO.output(BUZZER_PIN, GPIO.LOW)  # Turning off the buzzer
                print('Buzzer Off')

        time.sleep(0.1)
except KeyboardInterrupt:
    print("Exiting...")
    GPIO.cleanup()
