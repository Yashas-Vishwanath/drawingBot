import roslibpy
import time

client = roslibpy.Ros(host='192.168.11.79', port=9090)
client.run()

talker = roslibpy.Topic(client, '/chatter', 'std_msgs/String')

while client.is_connected:
    talker.publish(roslibpy.Message({'data': 'Hello, World!'}))
    print('Sending message...')
    time.sleep(3)

talker.unadvertise()

client.terminate()