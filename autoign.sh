#!/bin/bash

# Set ROS Distribution (validate or modify as needed)
ROS_DISTRO=${ROS_DISTRO:-"humble"}  # Default to humble if not set

# Update the system and install essential packages
echo "Updating system and installing essential tools..."
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    gnupg2 \
    lsb-release \
    unzip \
    python3-pip \
    python3-colcon-common-extensions \
    bash-completion \
    libgz-cmake3-dev \
    libgz-sim7-dev

# Add Gazebo repository for Fortress
echo "Adding Gazebo Fortress repository..."
sudo rm -f /etc/apt/sources.list.d/gazebo-stable.list
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update package lists and install Gazebo Fortress
echo "Installing Gazebo Fortress and related packages..."
sudo apt-get update
sudo apt-get install -y ignition-fortress ros-$ROS_DISTRO-ros-gz

# Install ROS 2 Gazebo Integration packages in a single command
echo "Installing ROS 2 Gazebo Integration and RViz2 packages..."
sudo apt-get install -y \
    ros-$ROS_DISTRO-ros-gz-bridge \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-ros-gz-interfaces \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-rviz2

# Final update and cleanup
echo "Performing final cleanup..."
sudo apt-get autoremove -y
sudo apt-get clean

# Source the ROS environment
echo "Sourcing ROS setup file..."
source /opt/ros/$ROS_DISTRO/setup.bash

echo "Setup complete! Gazebo Fortress and ROS 2 integration are installed."
echo "You can start Gazebo with the command 'ign gazebo'."
echo "You can run RViz2 using the 'rviz2' command."
