import time
import json
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
# Set up AWS IoT client
iot_client = AWSIoTMQTTClient("AWSPublisher, 0")
iot_client.configureEndpoint("a2yfm7zykvqeaz-ats.iot.us-east-1.amazonaws.com", 8883)
iot_client.configureCredentials("AmazonRootCA1.pem", "private.pem.key", "certificate.pem")
iot_client.connect()

coordinates_str1 = input("Enter the coordinates separated by comma (e.g., x,y): ")
H1, V1 = map(int, [coord[1:] for coord in coordinates_str1.split(',')])
while True:
    coordinates_str2 = input("Enter the coordinates separated by comma (e.g., x,y): ")
    H2, V2 = map(int, [coord[1:] for coord in coordinates_str2.split(',')])
    if 0 <= H2 <= 6 and 0 <= V2 <= 6:
        break
    else:
        print("Invalid coordinates! Please enter coordinates within the range H0,V0 to H6,V6.")
seq_commands = [
        {"linear_velocity": 0, "angular_velocity": 0},
        {"linear_velocity": 1.2, "angular_velocity": 0},  # Move forward
        {"linear_velocity": 0, "angular_velocity": 0},  # Stop
        ]
def negative_angular_velocity():
    initial_command = {"linear_velocity": 0, "angular_velocity": -1.2}
    initial_command_json = json.dumps(initial_command)
    iot_client.publishAsync(topic='robot/control', payload=initial_command_json, QoS=1)
    print(f"Published initial command: {initial_command_json}")

def positive_angular_velocity():
    initial_command = {"linear_velocity": 0, "angular_velocity": 1.2}
    initial_command_json = json.dumps(initial_command)
    iot_client.publishAsync(topic='robot/control', payload=initial_command_json, QoS=1)
    print(f"Published initial command: {initial_command_json}")

def stop_robot():
    stop_command = {"linear_velocity": 0, "angular_velocity": 0}
    stop_command_json = json.dumps(stop_command)
    iot_client.publishAsync(topic='robot/control', payload=stop_command_json, QoS=1)
    print(f"Published stop command: {stop_command_json}")

#starting and target positions
start_pos = (H1,V1)
target_pos = (H2, V2)

# Cal. The distance between start and target positions
num_steps = int(distance / step_distance)if distance != 0 else 0
x_increment = 0
y_increment = 0
if num_steps == 0:
    pass
else:
    # Calculate the x and y increments for each step
    x_increment = (target_pos[0] - start_pos[0]) / num_steps
    y_increment = (target_pos[1] - start_pos[1]) / num_steps

# Print the coordinates of each step
for i in range(num_steps + 1):
    x = start_pos[0] + i * x_increment
    y = start_pos[1] + i * y_increment

if V1 == V2 and H1 != H2 and H2 > H1:
    print('Entered into V1 == V2 and H1 != H2 and H2 > H1 condition')
    for i in range(abs(H2 - H1)):
        H = H1 + i * (1 if H2 > H1 else -1)
        print(f"Robot is at H{H},V{V1} position")
        command_json = json.dumps({"linear_velocity": 1.2, "angular_velocity": 0})  # Move forward command
        iot_client.publish(topic='robot/control', payload=command_json, QoS=1)
        time.sleep(1.7)  # Assuming 1 step = 1.6 seconds
    print(f"Robot reached the target position H{H2}, V{V2}. Stopping the robot.")
    stop_robot()
    #stop_command_json = json.dumps(seq_commands[0])  # Stop command
    #iot_client.publishAsync(topic='robot/control', payload=stop_command_json, QoS=0) 
elif H1 == H2 and V1 != V2 and V2 > V1:
    print('Entered into H1 == H2 and V1 != V2 and V2 > V1 condition')
    positive_angular_velocity()
    time.sleep(2)
    for i in range(abs(V2 - V1)):
        V = V1 + i * (1 if V2 > V1 else -1)# it increments H by 1 in each step
        print(f"Robot is at H{H1},V{V} position")
        command_json = json.dumps(seq_commands[1])  # Move forward command
        iot_client.publishAsync(topic='robot/control', payload=command_json, QoS=1)
        time.sleep(1.75)  # Assuming 1 step = 1.6 seconds
    print(f"Robot reached the target position H{H2}, V{V2}. Stopping the robot.")
    stop_command_json = json.dumps(seq_commands[0])  # Stop command
    iot_client.publishAsync(topic='robot/control', payload=stop_command_json, QoS=1)

elif H1 != H2 and V1 != V2 and H1 == V1 == 0:
    print('Entered into H1 != H2 and V1 != V2 and H1 == V1 == 0 condition')
    for i in range(abs(H2 - H1)):
        H = H1 + i * (1 if H2 > H1 else -1)
        print(f"Robot is at H{H},V{V1} position")
        command_json = json.dumps(seq_commands[1])  # Move forward comman
        iot_client.publishAsync(topic='robot/control', payload=command_json, QoS=1)
        time.sleep(1.8)  # Assuming 1 step = 1.6 seconds
    print(f"Robot is at H{H2},V{V1} position")
    positive_angular_velocity()
    time.sleep(1.8)
    for i in range(abs(V2 - V1)):
        V = V1 + (i+1) * (1 if V2 > V1 else -1)
        print(f"Robot is at H{H2},V{V} position")
        command_json = json.dumps(seq_commands[1])  # Move forward command
        iot_client.publishAsync(topic='robot/control', payload=command_json, QoS=1)
        time.sleep(1.9)  # Assuming 1 step = 1.6 seconds
    print(f"Robot reached the target position H{H2}, V{V2}. Stopping the robot.")
    stop_command_json = json.dumps(seq_commands[0])  # Stop command
    iot_client.publishAsync(topic='robot/control', payload=stop_command_json, QoS=1)
elif V1 == V2 and H1 != H2 and H2 < H1:
    print('Entered into V1 == V2 and H1 != H2 and H2 < H1 condition')
    positive_angular_velocity()
    time.sleep(3.53)
    for i in range(abs(H1 - H2)):
        H = H1 - i
        print(f"Robot is at H{H},V{V1} position")
        command_json = json.dumps(seq_commands[1])  # Move forward command
        iot_client.publishAsync(topic='robot/control', payload=command_json, QoS=1)
        time.sleep(2.2)  # Assuming 1 step = 1.6 seconds
    print(f"Robot reached the target position H{H2}, V{V2}. Stopping the robot.")
    stop_command_json = json.dumps(seq_commands[0])  # Stop command
    iot_client.publishAsync(topic='robot/control', payload=stop_command_json, QoS=1)

elif H1 == H2 and V1 != V2 and V1 > V2:
    print('Entered into H1 == H2 and V1 != V2 and V1 > V2 condition')
    negative_angular_velocity()
    time.sleep(2.0)
    for i in range(abs(V1 - V2)):
        V = V1 - i
        print(f"Robot is at H{H1},V{V} position")
        command_json = json.dumps(seq_commands[1])  # Move forward command
        #time.sleep(2)  # Wait for 2 seconds before the robot starts moving
        iot_client.publishAsync(topic='robot/control', payload=command_json, QoS=1)
        time.sleep(2.3)  # Assuming 1 step = 1.6 seconds
    print(f"Robot reached the target position H{H2}, V{V2}. Stopping the robot.")
    stop_command_json = json.dumps(seq_commands[0])  # Stop command
    iot_client.publishAsync(topic='robot/control', payload=stop_command_json, QoS=1)

elif V2 > V1 and H1 > H2:
    print('Entered into V2 > V1 and H1 > H2 condition')
    positive_angular_velocity()
    time.sleep(1.5)
    for i in range(abs(V2 - V1)):
        V = V1 + i
        print(f"Robot is at H{H1},V{V} position")
        command_json = json.dumps(seq_commands[1])  # Move forward command
        iot_client.publishAsync(topic='robot/control', payload=command_json, QoS=1)
        time.sleep(1.8)
    positive_angular_velocity()
    time.sleep(1.5)	
	for i in range(abs(H1 - H2)):
        H = H1 - i
        print(f"Robot is at H{H},V{V2} position")
        command_json = json.dumps(seq_commands[1])  # Move forward command
        iot_client.publishAsync(topic='robot/control', payload=command_json, QoS=1)
        time.sleep(1.9)  # Assuming 1 step = 1.6 seconds
    print(f"Robot reached the target position H{H2}, V{V2}. Stopping the robot.")
    stop_command_json = json.dumps(seq_commands[0])  # Stop command
    iot_client.publishAsync(topic='robot/control', payload=stop_command_json, QoS=1)

elif V1 > V2 and H1 > H2:
    print('Entered into V1 > V2 and H1 > H2 condition')
    negative_angular_velocity()
    time.sleep(1.95)
    for i in range(abs(V1 - V2)):
        V = V1 -i
        print(f"Robot is at H{H1},V{V} position")
        command_json = json.dumps(seq_commands[1])  # Move forward command
        iot_client.publishAsync(topic='robot/control', payload=command_json, QoS=1)
        time.sleep(2)  # Assuming 1 step = 1.6 seconds
    negative_angular_velocity()
    time.sleep(1.95)
    for i in range(abs( H1 - H2)):
        H = H1 - i
        V = -(V1- V2 - V1)
        print(f"Robot is at H{H},V{V} position")
        command_json = json.dumps(seq_commands[1])  # Move forward command
        iot_client.publishAsync(topic='robot/control', payload=command_json, QoS=1)
        time.sleep(1.8)  # Assuming 1 step = 1.6 seconds
    print(f"Robot reached the target position H{H2}, V{V2}. Stopping the robot.")
    stop_command_json = json.dumps(seq_commands[0])  # Stop command
    iot_client.publishAsync(topic='robot/control', payload=stop_command_json, QoS=1)
elif V1 > V2 and H1 < H2:
    print('Entered into V1 > V2 and H1 < H2 condition ')
    for i in range(abs(H2 - H1)):
        H = H1 + i
        print(f"Robot is at H{H},V{V1} position")
        command_json = json.dumps(seq_commands[1])  # Move forward command
        iot_client.publishAsync(topic='robot/control', payload=command_json, QoS=1)
        time.sleep(1.8)  # Assuming 1 step = 1.6 seconds
    negative_angular_velocity()
    time.sleep(2.1)
    for i in range(abs(V1 - V2)):
        V = V1 - i
        print(f"Robot is at H{H2},V{V} position")
        command_json = json.dumps(seq_commands[1])  # Move forward command
        iot_client.publishAsync(topic='robot/control', payload=command_json, QoS=1)
        time.sleep(1.75)  # Assuming 1 step = 1.6 seconds
    print(f"Robot reached the target position H{H2}, V{V2}. Stopping the robot.")
    stop_command_json = json.dumps(seq_commands[0])  # Stop command
    iot_client.publishAsync(topic='robot/control', payload=stop_command_json, QoS=1)

elif H2 >= H1 and V1 <= V2:
    print('Entered into H2 >= H1 and V1 <= V2 condition')
    positive_angular_velocity()
    time.sleep(1.95)
    for i in range(abs(V2 - V1)):
        V = V1 + i
        print(f"Robot is at H{H1},V{V} position")
        command_json = json.dumps(seq_commands[1])  # Move forward command
        iot_client.publishAsync(topic='robot/control', payload=command_json, QoS=1)
        time.sleep(2)  # Assuming 1 step = 1.6 seconds
    negative_angular_velocity()
    time.sleep(1.95)
    for i in range(abs(H2 - H1)):
        H = H1 + i
        print(f"Robot is at H{H},V{V2} position")
        command_json = json.dumps(seq_commands[1])  # Move forward command
        iot_client.publishAsync(topic='robot/control', payload=command_json, QoS=1)
        time.sleep(1.8)  # Assuming 1 step = 1.6 seconds
    print(f"Robot reached the target position H{H2}, V{V2}. Stopping the robot.")
    stop_command_json = json.dumps(seq_commands[0])  # Stop command
    iot_client.publishAsync(topic='robot/control', payload=stop_command_json, QoS=1)

stop_robot()
time.sleep(2)