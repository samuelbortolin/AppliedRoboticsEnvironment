# Install and run the simulator on a clean Ubuntu 16.04 installation

## How to install ROS Kinetic 
Reference: [http://wiki.ros.org/kinetic/Installation/Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu)

Setup:

```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop-full
```

Environment setup:

```
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

Dependencies for building packages:

```
$ sudo apt install python-rosdep
$ sudo apt install ros-kinetic-rqt-multiplot ros-kinetic-usb-cam
$ sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
$ sudo apt-get install python-catkin-tools
$ sudo apt install chrpath
$ sudo apt-get install libignition-math2-dev
$ sudo apt install terminator # package to manage multiple terminal session
$ sudo apt install ros-kinetic-jsk-visualization # tool to visualize the detected polygon in RVIZ
```

Initialize rosdep:

```
$ sudo rosdep init
$ rosdep update
```

## Hot to install Gazebo 7.16
Rereference: [http://gazebosim.org/tutorials?tut=install_ubuntu&ver=7.0&cat=install](http://gazebosim.org/tutorials?tut=install_ubuntu&ver=7.0&cat=install)

Setup:

```
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Install Gazebo:

```
$ sudo apt-get update
$ sudo apt-get install gazebo7
```

Check installation:

```
$ gazebo
```

Solve known issue with VMware

```
$ echo "export SVGA_VGPU10=0" >> ~/.profile
```

