# UAV-RT Connection package

This ROS 2 Node is responsible for establishing and monintoring a UDP or serial onnection with the PX4, as well as encoding and decoding MAVLink messages that are passed through the companion computer [PX4 autopilot](https://docs.px4.io/master/en/flight_controller/pixhawk4.html) (with a [SiK 915 MHz telemetry radio](https://ardupilot.org/copter/docs/common-sik-telemetry-radio.html#sik-telemetry-radio)) to the ground control station (GCS) PX4 (also connected to a 915 MHz radio). The GCS is running a custom version of QGroundControl (QGC) developed by [Don Gagne](https://github.com/DonLakeFlyer).

The development of this code was funded via National Science Foundation grant no. 2104570.

# Features

Note: These features need to be updated. I will do this once I start the UAV-RT documenation, since text from that documentation will end up here. 

- Establishes and monitors a serial or UDP connection with a [PX4 autopilot](https://docs.px4.io/master/en/flight_controller/pixhawk4.html) or [Gazebo SITL](https://ardupilot.org/dev/docs/using-gazebo-simulator-with-sitl.html)
  - This connection is established and utilized with [MAVSDK C++](https://mavsdk.mavlink.io/main/en/cpp/)
- Recieves and publishes telemetry data from the PX4 at a rate of 2Hz
  - This data is published to the /antennaPose topic
  - This data is written to a dynamic array
  - Able to write this telemetry data to a .txt file for post-processing
- Performs [interpolation](https://en.wikipedia.org/wiki/Linear_interpolation) and [slerp](https://en.wikipedia.org/wiki/Slerp#Quaternion_Slerp) for determining the position of the UAV in space given a timestamp
  - Pulse data is generated/supplied by UAV-RT detectors
    - This data is collected through a subscriber that is subscribed to the /pulse topic
  - Position and quaterion is sourced from the telemetry data collected from the PX4
  - Publishes the pulse and interpolated/slerped data to the /pulsePose topic
 - Encodes pulse and interpolated/slerped data so that it can be sent as a MAVLink message to QGC
 - Decodes messages MAVLink message sent from QGC and forwards them to UAV-RT/Supervisor
  - Supported messages are outlined in the "Tag Interface Control" document
 - The messages are stored within MAVLink's [DEBUG_FLOAT_ARRAY #350](https://mavlink.io/en/messages/common.html#DEBUG_FLOAT_ARRAY)

# Documentation

The supporting documentation for this project can be found here. (TBD)

# System requirements

The system requirments for the use of this package can be found here. (TBD)

# Installaton

This installation assumes that you have completed the installation process for uavrt_source. 

### Linux

Within a terminal window, run the following commands:

```
source /opt/ros/galactic/setup.bash
cd ~/uavrt_workspace/uavrt_source/
```

You must be a member of the Dynamic and Active Systems Lab organization on Github. Authentication is currently required for the following command: 

```
git clone https://github.com/dynamic-and-active-systems-lab/uavrt_conneciton/
cd ~/uavrt_workspace/
```

"All required rosdeps installed successfully" should be returned after the following command: 

```
rosdep install -i --from-path uavrt_source --rosdistro galactic -y
```

The following command will only build out the uavrt_connection package. This is done to isolate errors or warnings: 

```
colcon build --packages-select uavrt_connection
source /opt/ros/galactic/setup.bash
. install/local_setup.bash
```

If these commands didn't fail, then your installation of uavrt_connection should be complete. 

# Basic operation

In order to run the uavrt_connection package in isolation, use the following command: 

```
ros2 run uavrt_connection main #
```

Where # corresponds to the type of connection you will be using. 

Enter a '0' to use a serial connection or enter a '1' to use an UDP connection.
For Serial, the connection string will be: 'serial:///dev/ttyACM0', and expects a PX4 autopilot to be plugged into this port. 
For UDP, the connection string will be: 'udp://:14540', and expects a Gazebo SITL to be running at this port. 

# Troubleshooting

Troubleshooting tips can be found here. (TBD)

# License

This codebase is released under the GNU Lesser General Public License v3 or later.
