import serial
import time
from azure.iot.device import IoTHubDeviceClient, Message

# Create a serial object
ser = serial.Serial(port='/dev/ttyS0', baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)
class AzureIoTClient:
    def __init__(self):
        self.init_azure_iot()

    def init_azure_iot(self):
        #connection_str = "HostName=FMRIoTHub1.azure-devices.net;DeviceId=RosberryPi;SharedAccessKey=3vy797qXXAanvQE+YQKOL9+d6cgLtk3fCAIoTC4ocXs=" #Subrama
        connection_str = "HostName=IoThub7.azure-devices.net;DeviceId=IoTRobotdevice2;SharedAccessKey=GrcSn2sHhJvIhDqWguWE+tZnouEexaKwOAIoTM2DmCs=" # Venkat
        self.device_client = IoTHubDeviceClient.create_from_connection_string(connection_str)
        #self.device_client.on_message_received = self.message_received_handler

    def send_message_to_azure(self, message_body):
        message = Message(message_body)
        self.device_client.send_message(message)

    #def message_received_handler(self, message):
        #print("Message received from Azure IoT Hub:", message)


azure_client = AzureIoTClient()
try:
    while True:
        # Read a line from the serial port
        line = ser.readline().decode('utf-8').strip()

        # Check if the line contains the battery percentage
        if "Percentage" in line:
            # Extract the battery percentage
            pct = float(line.split(":")[1].split("%")[0].strip())
            print(f"Battery Percentage: {pct}%")
            message_body = f"Battery Percentage: {pct}%"
            azure_client.send_message_to_azure(message_body)
        elif "Voltage" in line:
            # Extract the battery voltage
            voltage = float(line.split(":")[1].replace('V', '').strip())
            print(f"Battery Voltage: {voltage}V")
            #message_body = f"Battery Voltage: {voltage}V"

            # Send battery voltage to Azure IoT Hub
            #azure_client.send_message_to_azure(message_body)
        # Delay before next iteration
        time.sleep(1)
except KeyboardInterrupt:
    print("Exiting...")
