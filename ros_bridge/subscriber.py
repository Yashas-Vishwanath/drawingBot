# # write a subscriber node using roslibpy that subscribes to /scan topic and prints the data to the console

# import roslibpy

# client = roslibpy.Ros(host='192.168.11.79', port=9090)
# client.run()

# subscriber = roslibpy.Topic(client, '/scan', 'sensor_msgs/LaserScan')
# subscriber.subscribe(lambda message: print(message))

# try:
#     while client.is_connected:
#         pass
# except KeyboardInterrupt:
#     subscriber.unsubscribe()
#     client.terminate()



# print only the first message from the topic and then unsubscribe

import roslibpy

# Flag to track if the first message has been received
first_message_received = False

# Callback function to handle incoming messages
def message_callback(message):
    global first_message_received
    if not first_message_received:
        # Print the first message
        print("First message received:", message)
        print("Values in 'ranges' are:", len(message['ranges']))
        # Update the flag to indicate that the first message has been received
        first_message_received = True

# Create ROS client and connect to ROS environment
client = roslibpy.Ros(host='192.168.11.79', port=9090)
client.run()

# Create a subscriber for the /scan topic
subscriber = roslibpy.Topic(client, '/scan', 'sensor_msgs/LaserScan')

# Subscribe to messages on the /scan topic
subscriber.subscribe(message_callback)

try:
    # Keep the script running until interrupted by keyboard
    while client.is_connected and not first_message_received:
        pass
except KeyboardInterrupt:
    # Unsubscribe from the /scan topic
    subscriber.unsubscribe()
    # Terminate the ROS client connection
    client.terminate()
