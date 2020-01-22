# Meta Package for the EPOCHS Reference Application (ERA)

This is the package to get if you want to install ERA. The rosinstall script will fetch the necessary packages in the catkin workspace.

### We recommend installing Ubuntu 18.04 with ROS Melodic. Installation instructions can be found <a href="https://github.com/IBM/era/wiki/ERA-on-Ubuntu-18.04-with-ROS-Melodic" target="_blank">here</a>


## Requirements

ERA requires:
 - Ubuntu 16.04 (Check out wiki for 18.04 installation)
 - Python 2.7
 - GNU gcc/g++ 5.4
 - cmake 3.5.0+
 - python-wstool (`sudo apt-get install python-wstool`)
 - ROS Kinetic and Gazebo 7 (Check out wiki for ROS Melodic and Gazebo 9 installation)
 - GNU Radio 3.7.3+
 - <a href="https://github.com/bastibl/gr-ieee802-11" target="_blank">gr-ieee802-11</a>
 - Swig 3.0
 - LZ4 1.7.0+

**Note that gr-ieee802-11 is included as a package in this project (i.e. no need to clone it separately).**
 

Follow <a href="http://wiki.ros.org/kinetic/Installation/Ubuntu" target="_blank">these</a> instructions in ROS' official website to install ROS Kinetic and Gazebo 7 on Ubuntu 16.04. We recommend installing _desktop-full install_ (`ros-kinetic-desktop-full`).

These additional ROS packages are also required:
 - ros-kinetic-turtlebot
 - ros-kinetic-kobuki
 - ros-kinetic-robot-state-publisher
 - ros-kinetic-robot-pose-publisher
 - ros-kinetic-joint-state-publisher
 - ros-kinetic-costmap-2d

Some subset of these additional ROS packages is required to teleoperate the robots and perform sensing:
 - ros-kinetic-kobuki-controller-tutorial
 - ros-kinetic-kobuki-core
 - ros-kinetic-kobuki-description
 - ros-kinetic-kobuki-desktop
 - ros-kinetic-kobuki-gazebo
 - ros-kinetic-kobuki-gazebo-plugins
 - ros-kinetic-kobuki-keyop
 - ros-kinetic-kobuki-node

The following command should install all these required dependencies:

```
sudo apt-get install ros-kinetic-turtlebot ros-kinetic-kobuki ros-kinetic-robot-state-publisher ros-kinetic-robot-pose-publisher ros-kinetic-joint-state-publisher ros-kinetic-costmap-2d ros-kinetic-kobuki-controller-tutorial ros-kinetic-kobuki-core ros-kinetic-kobuki-description ros-kinetic-kobuki-desktop ros-kinetic-kobuki-gazebo ros-kinetic-kobuki-gazebo-plugins ros-kinetic-kobuki-keyop ros-kinetic-kobuki-node
```


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

(it may be necessary to prepend `DISPLAY=:0` to the command above in some cases)

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

Finally, start `roscore.service`:

```
sudo systemctl start roscore.service
```


## Profiling ERA

We provide support for ERA profiling through two complementary approaches: GNU Radio's _performance counters_ [1][2], and Linux _perf_ [3].

### GNU Radio's Performance Counters

GNU Radio implements _performance counters_ that can be used to capture/monitor information about a block inside a running flowgraph [1][2]. To enable them, GNU Radio has to be compiled from sources with specific `cmake` flags. Please, refer to the <a href="https://github.com/IBM/era/wiki#enabling-gnu-radios-performance-counters" target="_blank">ERA Wiki</a> for detailed information. In addition, performance counters has to be enabled in `${prefix}/etc/gnuradio/conf.d/gnuradio-runtime.conf`, where `${prefix}` is GNU Radio's installation directory (e.g. `/opt/gnuradio/`):

```
[PerfCounters]
on = True
export = True
clock = thread

[ControlPort]
on = True
edges_list = True
```

At runtime, performance counters can be collected using the `gr-perf-to-csv` script in <a href="https://github.com/IBM/dsrc/tree/master/gr-foo/utils" target="_blank">dsrc/gr-foo/utils</a>. For example, once ERA is steadily running, execute the following commands in a different terminal:

```
cd ~/catkin_ws/src/dsrc/gr-foo/utils/
./gr-perf-to-csv 127.0.0.1 <port>
```

where `<port>` is the Apache Thrift port through which the running flowgraph publishes its counters. This port number is shown during the initialization of ERA, with a message similar to this one:
```
gr::log :INFO: controlport - Apache Thrift: -h host_name -p port_number
```


### Linux perf

GNU Radio's performance counters provide coverage only for GNU Radio flowgraphs. In order to collect end-to-end ERA profiling data, we also support Linux perf [3]. Once ERA is steadily running, execute the following commands in a different terminal:

```
cd ~/catkin_ws/src/era_gazebo/utils/
./profile_era.py
```

The `profile_era.py` script (<a href="https://github.com/IBM/era_gazebo/tree/master/utils" target="_blank">era_gazebo/utils</a>) invokes Linux perf as root. It can also be invoked without root privileges, but in this case the profiling data could lack of some kernel-related information. Also, note that the execution of `profile_era.py` has to be manually stopped by the user (Ctrl-C).

Linux perf generates a binary file (`output.perf`) that can be analyzed with the following command:

```
sudo perf report -g -i output.perf
```


## Moving and Controlling the TurtleBots

As an alternative to the use of bag files, as explained before, the robots can be teleoperated using the keyboard. Each robot is controlled independently; therefore, for each one, open a separate terminal and execute the following command:

```
roslaunch era_gazebo keyboard_teleop.launch namespace:=<robot_id>
```

Replace `<robot_id>` with the ID of the robot to be controlled, like `r0` and `r1`.


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


## Contributors

Augusto Vega (IBM)  
Akin Sisbot (IBM)  


## Current Maintainers

ajvega@us.ibm.com  
easisbot@us.ibm.com  


## FAQs



## References

[1] <a href="https://wiki.gnuradio.org/index.php/PerformanceCounters" target="_blank">https://wiki.gnuradio.org/index.php/PerformanceCounters</a>

[2] T. Rondeau, T. O'Shea, and N. Goergen. "Inspecting GNU Radio Applications with Controlport and Performance Counters." In Proceedings of the second workshop on Software radio implementation forum (SRIF '13). 2013.

[3] <a href="https://perf.wiki.kernel.org/index.php/Main_Page" target="_blank">https://perf.wiki.kernel.org/index.php/Main_Page</a>
