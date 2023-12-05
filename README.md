# Robot Interface EKI

The objective of this package is to send commands to move the robotic arm and the gripper in an easy way through a python script in ROS2 Dashing.

In the following figure you can see the basic principle of operation of this package:


## How to install:


1. Install ROS2 via debian packages for Ubuntu Bionic (18.04).

    Instruction in the ROS2 wiki https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/

2. Create a workspace.

    Instructions in the ROS2 wiki https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/

3. Clone this repo to `<ros2-workspace>/src`

```bash
git clone https://www.w.hs-karlsruhe.de/gitlab/robolab/ros/robot_interface_eki.git
```

## How to launch:


1. Before starting it is necessary to configure the host, port and meta_port parameters of the robot_interface_launch.py file in order to communicate with the robot. Make sure, it has the next values:

```python
    parameters=[
        {'host': '10.166.32.145'},
        {'port': 54600},
        {'meta_port': 54601}
    ],
    output='screen'
```

2. This following command is only used for the package to recognize a library that it uses internally: 

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/<user_name>/<workspace>/build/robot_interface_eki/lib
```

3. build the packages in `<ros2-workspace>/src`

```bash
colcon build
```

4. always source in each shell:

```bash
source /opt/ros/dashing/setup.bash
source ~/<workspace>/install/setup.bash
```

5. In first shell:

```bash
ros2 launch robot_interface_eki robot_interface.launch.py
```

This will start the server with the previous network configuration (add "stdbuf -o L" at the beginning just if you want to see the logs messages too)

6. In second shell:

```bash
ros2 run robot_interface_eki client.py
```

This file is the one you must modify to send commands to the robot. There you will also find several examples so you can get started quickly.


# Testing

Test if port is open
```bash
sudo hping3 -S -p 54601 10.166.32.145
```
if the reply has flags=RA, you received a reset response, so the port is closed, but if you receive flags=SA, the port is open

