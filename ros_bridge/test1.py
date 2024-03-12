import roslibpy

client = roslibpy.Ros(host='192.168.11.79', port=9090)
client.run()

print("is ros connected?", client.is_connected)
client.terminate()

