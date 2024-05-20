set -e

# setup ros environment
source "/opt/ros/noetic/setup.bash"
source "/dev_ws/devel/setup.bash"

# Set permissions for the serial port
if [ -e /dev/ttyACM0 ]; then
    chmod 666 /dev/ttyACM0
else
    echo "Device /dev/ttyACM0 not found. Make sure the Arduino is connected."
fi

export ROS_MASTER_URI=http://10.41.1.1:11311
export ROS_IP=10.41.1.1
exec "$@"
