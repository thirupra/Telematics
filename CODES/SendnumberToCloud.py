import time
from azure.iot.device import IoTHubDeviceClient, Message

# Connection string for your device (replace with your values)
connection_string = "HostName=iothub4.azure-devices.net;DeviceId=IoTRobotDevice1;SharedAccessKey=MqXdalhNRRIAO5ZPrroC+t5Ru0/aZqkxBAIoTOnUklI="

# Create an IoT Hub client
client = IoTHubDeviceClient.create_from_connection_string(connection_string)

# Connect to Azure IoT Hub
client.connect()

try:
    for i in range(1, 101):
        # Send a message to Azure IoT Hub with a sequence number
        #message = Message("{}".format(i))
        message = Message(i)
        client.send_message(message)
        print("Message {} sent to Azure IoT Hub".format(i))

        # Wait for a while before sending the next message (e.g., every 5 seconds)
        time.sleep(5)

except KeyboardInterrupt:
    print("Sample stopped by user")

finally:
    # Disconnect from Azure IoT Hub
    client.disconnect()
