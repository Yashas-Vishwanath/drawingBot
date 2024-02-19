# turtlebot remote

## Setup

### sourcing ROS

Add the following to your bashrc file for ros and the workspace to be sourced when opening a new terminal in dev_ws. make sure your are not sourcing ROS in your bashrc file.


```bash
if [ -f "/dev_ws/setup.bash" ]; then
    source /dev_ws/setup.bash
fi
```

## use

### build the image

Running the following command from the root of the repo will execute the build image shell script

```bash
.docker/build_image.sh
```

### run the image

Running the following command from the root of the repo will execute the run image shell script

```bash
.docker/run_user.sh
```

with Nvidia support (requires nvidia-docker2)

```bash
.docker/run_user_nvidia.sh
```
Once inside the container, ake ownership of the workspace with

```bash
sudo chown -R $USER /dev_ws
```

### terminal 

Terminator is installed in the container for multiple terminals launch terminator from the CLI inside the container. Run `terminator` to start.

## navigation

for gmapping run

```bash
roslaunch turtlebot_navigation gmapping_demo.launch
```

to sav map run
    
```bash
rosrun map_server map_saver -f ~/map
```

for navigation run (map in launch file)

```bash
roslaunch turtlebot_navigation amcl_demo_iaac.launch
```

