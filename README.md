**A ROS2 node for NMEA GPS**

This node is implementing basic telegrams for NMEA GPS communicating via serial lines (NMEA-0183). It publishes a message providing more information than the standard gps message (speed etc.). It also provides a service to read the observed satellites including signal strengthes.
It is currently developed and tested with ROS2 Foxy Fitzroy and Humble Hawksbill on Linux. Used devices were several Garmin and a u-blox based device.

**Build the node with:**

  ```colcon build```

**Run the node with:**

  ```ros2 run gpsx gps_node```

Additional parameters can be added for connection speed and connection port e.g.

```ros2 run gpsx gps_node --ros-args -p "comm_port:=/dev/ttyUSB1" -p "comm_speed:=9600"```
