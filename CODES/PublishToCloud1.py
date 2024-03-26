import warnings
from requests.packages.urllib3.exceptions import DependencyWarning

# Suppress only the InsecureRequestWarning from urllib3 needed for requests library
warnings.simplefilter('ignore', DependencyWarning)

import time
from azure.iot.device import IoTHubDeviceClient, Message

# Connection string for your device (replace with your values)
connection_string = "HostName=iothub4.azure-devices.net;DeviceId=IoTRobotDevice1;SharedAccessKey=hwLNqiJ1/D2ayEiPt9EGYIZiH4hTsQJ0GAIoTJb1vZo="

# Create an IoT Hub client
client = IoTHubDeviceClient.create_from_connection_string(connection_string)

# Connect to Azure IoT Hub
client.connect()

try:
    while True:
        # Send a message to Azure IoT Hub
        message = Message("Hello, Azure IoT!")
        client.send_message(message)
        print("Message sent to Azure IoT Hub")

        # Wait for a while before sending the next message (e.g., every 5 seconds)
        time.sleep(5)

except KeyboardInterrupt:
    print("Sample stopped by user")

finally:
    # Disconnect from Azure IoT Hub
    client.disconnect()
