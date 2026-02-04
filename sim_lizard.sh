#!/bin/bash
# Lizard Sim for Advanced UI
while true; do
  # GPS: RTK Fixed status (2) with coordinates in London
  ros2 topic pub -1 /ublox_gps_node/fix sensor_msgs/msg/NavSatFix "{status: {status: 2}, latitude: 51.539424, longitude: -0.118092}" > /dev/null 2>&1
  
  # Battery: 14.2V and 85%
  ros2 topic pub -1 /battery_state sensor_msgs/msg/BatteryState "{voltage: 14.2, percentage: 0.85}" > /dev/null 2>&1
  
  # E-Stop: False (Safe)
  ros2 topic pub -1 /estop1_state std_msgs/msg/Bool "{data: false}" > /dev/null 2>&1
  
  sleep 1
done
