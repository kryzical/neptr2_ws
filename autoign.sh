#!/bin/bash

# Update the system and install essential packages
echo "Updating system and installing essential packages..."
sudo apt-get update && sudo apt-get upgrade -y

# Install essential tools
echo "Installing essential tools..."
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
    libgz-*

sudo apt-get install libgz-cmake3-dev

# Add Gazebo repository for Fortress
echo "Adding Gazebo Fortress repository..."

# Remove existing conflicting sources (if any)
sudo rm -f /etc/apt/sources.list.d/gazebo-stable.list

# Add the GPG key and repository
echo "Fetching and adding the GPG key for Gazebo..."
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "Adding the Gazebo repository to the sources list..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update and install Gazebo Fortress
echo "Updating package lists and installing Gazebo Fortress..."
sudo apt-get update
sudo apt-get install -y ignition-fortress

# Install ROS 2 Gazebo packages
echo "Installing ROS 2 Gazebo packages..."
sudo apt-get install -y ros-$ROS_DISTRO-ros-gz

# install random bs for gazebo stuff
sudo apt-get install libgz-sim7-dev

# Install ROS 2 Gazebo Integration packages
echo "Installing ROS 2 Gazebo Integration packages..."
sudo apt-get install -y \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-interfaces

# Install joint_state_publisher_gui
echo "Installing joint_state_publisher_gui..."
sudo apt-get install -y ros-$ROS_DISTRO-joint-state-publisher-gui

# Final update and cleanup
echo "Performing final system upgrade and cleanup..."
sudo apt-get upgrade -y
sudo apt-get autoremove -y
sudo apt-get clean

echo "Setup complete! Gazebo Fortress is now installed."
echo "You can start Gazebo with the command 'ign gazebo'."

# Install RViz2
echo "Installing RViz2..."
sudo apt-get install -y ros-$ROS_DISTRO-rviz2

# Source the ROS environment
echo "Sourcing ROS setup file..."
source /opt/ros/$ROS_DISTRO/setup.bash

echo "RViz2 installation complete! You can run RViz2 using the 'rviz2' command."
