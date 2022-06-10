# UAV-RT MAVLink Encoder/Decoder package

This ROS 2 Node is responsible for encoding and decoding MAVLink messages that are passed through the companion computer [PX4 autopilot](https://docs.px4.io/master/en/flight_controller/pixhawk4.html) (with a 915 MHz radio) to the ground control station (GCS) PX4 (also connected to a 915 MHz radio). 

- The messages are stored within MAVLink's [DEBUG_FLOAT_ARRAY #350](https://mavlink.io/en/messages/common.html#DEBUG_FLOAT_ARRAY) 
- The GCS is running a custom version of QGroundControl (QGC) developed by [Don Gagne](https://github.com/DonLakeFlyer)

The development of this code was funded via National Science Foundation grant no. 2104570.

# Features

- Establishes and monitors a serial or UDP connection with a [PX4 autopilot](https://docs.px4.io/master/en/flight_controller/pixhawk4.html) or [Gazebo SITL](https://ardupilot.org/dev/docs/using-gazebo-simulator-with-sitl.html), respectively
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

# Basic operation

TBD

# Documentation

The supporting documentation for this project can be found on the following site:

TBD

# System requirements

The system requirments for the use of this package can be found on the following site:

TBD

# Installaton

TBD

# License

This codebase is released under the GNU Lesser General Public License v3 or later.

# License

This codebase is released under the GNU Lesser General Public License v3 or later.
