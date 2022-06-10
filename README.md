# uavrt_mavlink_encoder_decoder

This ROS 2 Node is responsible for encoding and decoding MAVLink messages that are passed through the [PX4 autopilot](https://docs.px4.io/master/en/flight_controller/pixhawk4.html). These messages are sent to and from the ground control station (GCS) running a custom version of Q Ground Control (QGC). 

The development of this code was funded via National Science Foundation grant no. 2104570.

# Features

- Establishes and monitors a serial or UDP connection with a [PX4 autopilot](https://docs.px4.io/master/en/flight_controller/pixhawk4.html) or [Gazebo SITL](https://ardupilot.org/dev/docs/using-gazebo-simulator-with-sitl.html), respectively
  - This connection is established and utilized with [MAVSDK C++](https://mavsdk.mavlink.io/main/en/cpp/)
- Recieves and publishes telemetry data from the PX4 at a rate of 2Hz
  - This data is published to the /antennaPose topic
  - Able to write this telemetry data to a .txt file for post-processing
- Performs [interpolation](https://en.wikipedia.org/wiki/Linear_interpolation) and [slerp](https://en.wikipedia.org/wiki/Slerp#Quaternion_Slerp) on pulse data generated by UAV-RT detectors and position/quaterion telemetry data collected from the PX4
  - Pulse data is collected through a subscriber that is subscribed to the /pulse topic
  - Publishes the interpolated and slerped data to the /pulsePose topic

# License

This codebase is released under the GNU Lesser General Public License v3 or later.
