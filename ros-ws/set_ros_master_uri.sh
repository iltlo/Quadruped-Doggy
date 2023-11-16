#!/bin/bash

# Get the current IP address
CURRENT_IP=$(hostname -I | awk '{print $1}')

# Set the ROS_MASTER_URI
export ROS_MASTER_URI="http://${CURRENT_IP}:11311"

# Display the updated ROS_MASTER_URI
echo "ROS_MASTER_URI is set to: $ROS_MASTER_URI"
