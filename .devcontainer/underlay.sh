#!/bin/bash

vcs import ./src < underlay.repos

source /opt/ros/"${ROS_DISTRO}"/setup.bash

colcon build --merge-install --packages-select cv_bridge