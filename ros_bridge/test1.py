# import roslibpy

# client = roslibpy.Ros(host='192.168.11.79', port=9090)
# client.run()

# print("is ros connected?", client.is_connected)
# client.terminate()



from __future__ import print_function
import roslibpy

client = roslibpy.Ros(host='192.168.11.79', port=9090)
client.on_ready(lambda: print('Is ROS connected?', client.is_connected))
client.run_forever()



# The difference between run() and run_forever() 
# is that the former starts the event processing in a separate thread, 
# while the latter blocks the calling thread.

