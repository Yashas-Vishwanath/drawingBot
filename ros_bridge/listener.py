import roslibpy

client = roslibpy.Ros(host='192.168.11.79', port=9090)
client.run()

listener = roslibpy.Topic(client, '/chatter', 'std_msgs/String')
listener.subscribe(lambda message: print('Received: ' + message['data']))

try:
    while True:
        pass
except KeyboardInterrupt:
    client.terminate()