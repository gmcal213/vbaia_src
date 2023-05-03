# VBAIA Team 23046 Project Code
Source code for Team 23046 - UArizona Senior Design 2022-23

These folders are downloadable ROS2 packages to hopefully be included in future robotics applications. Here are the steps to recreate and implement their functionality:
## Prerequisites
- Hardware Used
  - NVIDIA Jetson Nano
  - Oak-D-PoE (using PoE injector, need to split data and power lines)
  - Livox Mid-40
  - Network Switcher capable of at least 100MB/s, 1000MB/s preferred
- Software Installations
  - Jetson Nano flashed with Ubuntu 20.04, found [here](https://forums.developer.nvidia.com/t/xubuntu-20-04-focal-fossa-l4t-r32-3-1-custom-image-for-the-jetson-nano/121768)
  - ROS2 Foxy, installation instructions found [here](https://docs.ros.org/en/foxy/index.html)
  - Livox SDK, may be unnecessary to install, but included for brevity. Installation found [here](https://github.com/Livox-SDK/Livox-SDK)
  - Livox ROS2 Driver, installation instructions found [here](https://github.com/Livox-SDK/livox_ros2_driver)
  - DepthAI (Python package, install using pip)
  - Point Cloud Library (PCL), follow the tutorial [here](https://www.youtube.com/watch?v=VCobOzw2kHM)

## Setup
After following the Livox ROS2 driver install instructions in your home directory, it will create the folder "ws_livox". Go into the folder "ws_livox/src" and enter the following command:
```bash
git clone https://github.com/gmcal213/vbaia_src.git
```
This will copy the source code for each of the ROS2 Packages into the project, along with the Livox ROS2 Packages. You should now have in your src folder 4 sub folders
```
src
|---livox_ros2_driver
|---vbaia_launch
|---lidar_processor
|---oak-d-poe
```

Then, go into the project's home directory "ws_livox" and run
```bash
colcon build
```
This may take a long time the first time you compile. Highly advise adding more swap memory to the Nano or it will brick out while compiling. See tutorials on that 
[here](https://www.youtube.com/watch?v=JXv39FGi-nw&list=PLXYLzZ3XzIbgvysaNZn2IteAUCEgrIXNN&index=8) and [here](https://www.youtube.com/watch?v=wWr9HNsiOqo&list=PLXYLzZ3XzIbgvysaNZn2IteAUCEgrIXNN&index=26)

To setup the hardware, you need to connect the Nano to the Network Switch. Both the Oak-D-PoE and the Livox Mid-40 communicate over ethernet, and the Jetson Nano only has one ethernet port.
To do this, you need to configure the Livox Mid-40 to be in static mode. To do that, follow the instructions in the Livox Mid-40 [manual](https://www.oxts.com/wp-content/uploads/2021/08/Livox-Mid-Series-User-Manual-EN_compressed.pdf#Connection)

Then, you must set up the Nano with the correct static IP addresses:
- Address: 169.254.1.10, Netmask: 16
- Address: 192.168.1.40, Netmask: 24

## Using the Project
The whole workspace is divided into 4 ROS2 packages
- livox_ros2_driver
- oak_d_poe
- lidar_processor
- vbaia_launch

Each with its own list of executables. Run
```bash
ros2 pkg executables package_name
```
To get an idea of what each node is within the packages, see source code and comments for information about each function and its purpose. 

Before running any launch files or nodes of the system make sure you source your ROS2 environment! If you don't know what that means or why that's important, please read 
the ROS2 documentation about [configuring your environment](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html). Trust me, it will be
much more helpful for you in the long run.
To run the whole system (Oak-D-PoE running the on-board neural network, and the Livox Mid-40 performing hitch angle and pose calculations), enter the following command:
```bash
ros2 launch vbaia_launch vbaia_launch.py
```
You can check out the the "vbaia_launch.py" file for more information about the parameters that the launch file is setting.

To see the visual output of the system (see the output of the LiDAR plane segmentation and Oak-D-PoE neural network detections), enter the following command:
```bash
ros2 launch vbaia_launch visual_launch.py
```

You can check out the the "visual_launch.py" file for more information about the parameters that the launch file is setting.

## Contact
For more information about this project and access to more project documentation, feel free to shoot me a message at gavincaldwell317@gmail.com
