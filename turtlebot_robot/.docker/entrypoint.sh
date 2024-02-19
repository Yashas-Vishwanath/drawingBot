set -e

# setup ros environment
source "/opt/ros/noetic/setup.bash"
source "/dev_ws/devel/setup.bash"

export ROS_MASTER_URI=http://10.41.1.1:11311
export ROS_IP=10.41.1.1
exec "$@"