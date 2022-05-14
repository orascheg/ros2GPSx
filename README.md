# ros2GPSx
A ROS2 node for NMEA GPS

This node is implementing basic telegrams for NMEA GPS communicating via serial lines (NMEA-0183). It publishes a message providing more informaiton than the standard gps message (speed etc.)
It is currently developed and tested with ROS2 foxy on Linux

Build the node with:
colcon build

The node can be run via the ros2 run command

ros2 run gpsx gps_node

Additional parameters can be added for connection speed and connection port e.g.

ros2 run gpsx gps_node --ros-args -p "comm_port:=/dev/ttyUSB1" -p "comm_speed:=9600"
