#!/bin/bash

# Set ROS Distribution (default to Humble if not set)
ROS_DISTRO=${ROS_DISTRO:-"humble"}

# Exit immediately if a command fails
set -e

# Update the system and install essential packages
echo "Updating system and installing essential tools..."
sudo apt-get update -q
sudo apt-get install -yq \
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
    # Install Gazebo development dependencies
    libgz-cmake3-dev \
    libgz-sim7-dev \
    libgz-plugin2-dev  # Include necessary Gazebo plugin development libraries

# Add Gazebo repository for Fortress
echo "Adding Gazebo Fortress repository..."
sudo rm -f /etc/apt/sources.list.d/gazebo-stable.list
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Fortress and ROS 2 integration packages
echo "Installing Gazebo Fortress and ROS 2 integration..."
sudo apt-get update -q
sudo apt-get install -yq \
    ignition-fortress \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-ros-gz-bridge \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-ros-gz-interfaces \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-rviz2

# Perform cleanup
echo "Cleaning up unnecessary packages..."
sudo apt-get autoremove -yq
sudo apt-get clean -q

# Source the ROS setup file
echo "Sourcing ROS setup file..."
source /opt/ros/$ROS_DISTRO/setup.bash

# Inform user about the setup completion
echo "Setup complete! Gazebo Fortress and ROS 2 integration are installed."
echo "You can start Gazebo with: 'ign gazebo'."
echo "You can run RViz2 with: 'rviz2'."
