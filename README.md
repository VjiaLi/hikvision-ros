# Hikvision ROS2 Driver
ROS2 Driver Package for Hikvision Industrial Camera SDK


## Getting Started
To build the driver using ROS2 you need to clone the project into the `src` folder of a ros2
workspace as shown below:

```bash
mkdir -p hik_ws/src && cd hik_ws/src
git clone https://github.com/VjiaLi/hikvision-ros.git .
```

Next to compile the driver you need to source the ROS environemt into the active termainl:
```bash
source /opt/ros/<ros-distro>/setup.bash # replace ros-distro with 'rolling', 'humble', 'iron' or 'jazzy'
```

Finally, invoke `colcon build` command from within the catkin workspace as shown below:
```bash
cd hik_ws
colcon build
```

Once the build succeeds, you must source the _install_ folder of your ros2 workspace to add launch
commands to your environment:
```bash
source hik_ws/install/setup.bash
```

## Usage

### Only one camera
To connect to only a live camera you use the following launch file
```bash
ros2 launch hikvision-ros SingleCamera.launch.py    \
    params_file:=<path to params yaml file>
```

### Multiple camera
To connect to multiple live camera you use the following launch file
```bash
ros2 launch hikvision-ros MultiCamera.launch.py    \
    params_file:=<path to params yaml file>
```