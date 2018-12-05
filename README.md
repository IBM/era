# Meta Package for the EPOCHS Reference Application (ERA)

This is the package to get if you want to install ERA. The rosinstall script will fetch the necessary packages in the catkin workspace.


## Requirements

ERA requires:
 - Ubuntu 16.04
 - Python 2.7
 - GNU gcc/g++ 5.4
 - cmake 3.5.0+
 - python-wstool (`sudo apt get install python-wstool`)
 - ROS Kinetic and Gazebo 7
 - GNU Radio 3.7.3+
 - <a href="https://github.com/bastibl/gr-ieee802-11" target="_blank">gr-ieee802-11</a>
 - LZ4 1.7.0+

**Note that gr-ieee802-11 is included as a package in this project (i.e. no need to clone it separately).**
 

Required ROS packages:
 - kinetic-desktop-full
 - kinetic-turtlebot
 - kinetic-kobuki
 - kinetic-robot-state-publisher
 - kinetic-robot-pose-publisher
 - kinetic-joint-state-publisher
 - kinetic-costmap-2d

Some subset of these additional ROS packages is required to teleoperate the robots and perform sensing:
 - kinetic-kobuki-controller-tutorial
 - kinetic-kobuki-core
 - kinetic-kobuki-description
 - kinetic-kobuki-desktop
 - kinetic-kobuki-gazebo
 - kinetic-kobuki-gazebo-plugins
 - kinetic-kobuki-keyop
 - kinetic-kobuki-node


## A Note about GNU Radio and Its Built-In Profiling Support

GNU Radio implements _performance counters_ that can be used to capture/monitor information about a block inside a running flowgraph [1][2]. To enable them, GNU Radio has to be compiled from sources with specific `cmake` flags. Please, refer to the <a href="https://github.com/IBM/era/wiki#era-profiling" target="_blank">ERA Profiling</a> section in the ERA Wiki for detailed information.


## Installing ERA

If not done yet, source the ROS environment setup file:

```
source /opt/ros/kinetic/setup.bash
```

### Fresh Install with No Preexisting Catkin Workspace

```
cd ~
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
git clone https://github.com/IBM/era.git
cd ~/catkin_ws
wstool init src src/era/era.rosinstall
catkin_make
source devel/setup.bash
```

### Installation into an Existing Workspace

```
cd catkin_ws/src
git clone https://github.com/IBM/era.git
cd ~/catkin_ws
wstool merge -t src src/era/era.rosinstall
catkin_make
source devel/setup.bash
```

### Installation of GNU Radio Components

ERA includes components implemented in GNU Radio (gr-ros_interface, gr-foo and gr-ieee802-11) that have to be built and installed separately as explained next. The <a href="https://github.com/bastibl/gr-ieee802-11" target="_blank">gr-ieee802-11 project</a> is the GNU Radio-based implementation of an IEEE 802.11p transceiver while the <a href="https://github.com/bastibl/gr-foo" target="_blank">gr-foo project</a> includes a collection of custom blocks that are used by gr-ieee802-11. The gr-ros_interface project implements the interface to connect together the ROS and GNU Radio "worlds".

Build and install gr-ros_interface:

```
cd ~/catkin_ws/src/dsrc/gr-ros_interface/
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
```

Build and install gr-foo:

```
cd ~/catkin_ws/src/dsrc/gr-foo/
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
```

Build and install gr-ieee802-11:

```
cd ~/catkin_ws/src/dsrc/gr-ieee802-11/
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
```

Generate the GNU Radio Python flowgraph:

Finally, the GNU Radio-based implementation of the IEEE 802.11p transceiver has to be generated using the GNU Radio Companion compiler (`grcc`):

```
cd ~/catkin_ws/src/dsrc/gr-ieee802-11/examples
grcc ./wifi_transceiver.grc -d .
```


## Running ERA

To launch ERA (including Gazebo, ROS and GNU Radio):

```
roslaunch era_gazebo era.launch
```
Note: RViz will not run in a headless environment, so if you want to run in a visual environment, ensure that you launch the ROS workload from a desktop environment.

The launch command above starts Gazebo, ROS and GNU Radio, instantiates the robot(s), and remains waiting for user-provided commands to move the robots. In order to have a reproducible scenario, we recorded a random robot trajectory to a <a href="http://wiki.ros.org/Bags" target="_blank">bag file</a> that can be played back to recreate the robot's trip in the simulated world. To launch ERA and play back the bag file:

```
roslaunch era_gazebo era_playback.launch bag_name:=/*your_home_folder*/catkin_ws/src/era_gazebo/bagfiles/cmd_vel_r0.bag
```

To launch the workload **without the Gazebo and RViz GUIs**:

```
roslaunch era_gazebo era_playback.launch bag_name:=/*your_home_folder*/catkin_ws/src/era_gazebo/bagfiles/cmd_vel_r0.bag gui:=false
```

To launch the workload **enabling Linux perf profiling**:

```
roslaunch era_gazebo era_playback.launch bag_name:=/*your_home_folder*/catkin_ws/src/era_gazebo/bagfiles/cmd_vel_r0.bag gui:=false prof:=true
```

Although `roslaunch` will also start <a href="http://wiki.ros.org/roscore" target="_blank">roscore</a> (ROS's main engine required for _any_ ROS execution), a cleaner approach consists in having it running as an OS _daemon_ or service. Specifically for Ubuntu 15.04 and later, this can be done by creating a new <a href="https://wiki.ubuntu.com/SystemdForUpstartUsers" target="_blank">systemd</a> service called `roscore.service` in `/lib/systemd/system` (see code snippet below) and copying the `roscore_service.sh` script provided as part of <a href="https://github.com/IBM/era_gazebo" target="_blank">era_gazebo</a> to `/usr/local/bin`:

```
[Unit]
Description=start roscore
After=remote-fs.target
After=syslog.target

[Service]
ExecStart=/usr/local/bin/roscore_service.sh
Restart=on-abort

[Install]
WantedBy=multi-user.target
```


## Moving and Controlling the TurtleBots

As an alternative to the use of bag files, as explained before, the robots can be teleoperated using the keyboard. Each robot is controlled independently; therefore, for each one, open a separate terminal and execute the following command:

```
roslaunch era_gazebo keyboard_teleop.launch namespace:=<robot_id>
```

Replace `<robot_id>` with the ID of the robot to be controlled, like `r0` and `r1`.


## Automatically Profiling the ERA

To generate a sample workload using rosbag:

```
roslaunch era_gazebo era_auto.launch workload_path:=/path/to/bags workload_size:=[small|large] workload_type:=[rotator|wanderer] profiles_path:=/path/to/profiles project_path:=<default=.>
```

The duration of the recorded trace (bag) is determined by specifying `small` or `large` for the `workload_size` parameter. The behavior of the workload is determined by specifying `rotator` or `wanderer` for the `workload_type`. The `profiles_path` during this stage just sets up the correct directories for profiling. The `project_path` should point to the base directory of the ERA application.

To playback and profile a generated workload using rosbag and callgrind:

```
roslaunch era_gazebo era_auto_callgrind.launch workload_path:=/path/to/bags workload_size:=[small|large] workload_type:=[rotator|wanderer] profiles_path:=/path/to/profiles project_path:=<default=.>
```

The workload trace (bag) used to profile is determined by the `workload_size` and `workload_type` parameters. The output of the profiling (one per profiled node) is output to the `profiles_path`.


## Adjusting the Camera Frame Rate

In simulation mode (i.e. when using Gazebo to simulate the world and robots), the frame rate of the Kinect depth camera attached to Turtlebots can be adjusted by editing the `update_rate` element in the corresponding Xacro file (e.g. `turtlebot_gazebo.urdf.xacro`):

```
<xacro:macro name="turtlebot_sim_3dsensor">
  <gazebo reference="camera_link">  
    <sensor type="depth" name="camera">
      <update_rate>30.0</update_rate>
      ...
```

Also, be sure to set the plugin's update rate to 0 in order to allow the parent `<sensor>` tag to control the frame rate:

```
<plugin name="kinect_camera_controller" filename="libgazebo_ros_openni_kinect.so">
  <updateRate>0</updateRate>
  ...
```


## References

[1] <a href="https://wiki.gnuradio.org/index.php/PerformanceCounters" target="_blank">https://wiki.gnuradio.org/index.php/PerformanceCounters</a>

[2] T. Rondeau, T. O'Shea, and N. Goergen. "Inspecting GNU Radio Applications with Controlport and Performance Counters." In Proceedings of the second workshop on Software radio implementation forum (SRIF '13). 2013.
