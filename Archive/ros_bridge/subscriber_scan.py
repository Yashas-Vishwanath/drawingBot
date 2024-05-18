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




### print only the first message from the topic and then unsubscribe ###
### export the values in 'ranges' array and the corresponding angles to a csv file ###

import roslibpy
import csv
from datetime import datetime

# Flag to track if the first message has been received
first_message_received = False

# Callback function to handle incoming messages
def message_callback(message):
    global first_message_received
    if not first_message_received:
        # Print the first message
        # print("First message received:", message)

        # convert all the None values to 0 in the 'ranges' array
        message['ranges'] = [0 if x is None else x for x in message['ranges']]
        
        # round off all values in the 'ranges' array to 5 decimal places
        message['ranges'] = [round(x, 4) for x in message['ranges']]
        
        # print the values in 'ranges' array after converting None to 0
        print("Values in 'ranges' after converting None to 0 are:", message['ranges'])

        # get angles at which each measurement was taken using the 'angle_min' and 'angle_increment' fields
        angle_min = message['angle_min']
        angle_increment = message['angle_increment']
        num_readings = len(message['ranges'])
        # store the angles corresponding to each range measurement in a list
        angles = [round(angle_min + i * angle_increment, 4) for i in range(num_readings)]
        print("Angles in radians for each range:", angles)

        # print the number of values in 'ranges' array and the number of angles
        print("Number of values in 'ranges' array:", num_readings)
        print("Number of angles:", len(angles))

        # export values in 'ranges' array and the corresponding angles to a csv file
        # name the file with the current date and time
        current_date_time = datetime.now().strftime('%Y-%m-%d_%H-%M')
        file_name = 'laser_scan_' + current_date_time + '.csv'
        file_path = f'/home/yashas/drawingBot/ros_bridge/csvFiles/{file_name}'
        with open(file_path, 'w', newline='') as file:
            writer = csv.writer(file)
            # writer.writerow(['Angle (radians)', 'Range'])
            for i in range(num_readings):
                writer.writerow([angles[i], message['ranges'][i]])
        print(f"Data has been exported to {file_path}")

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
