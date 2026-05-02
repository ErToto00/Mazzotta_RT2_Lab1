#!/bin/bash

# Interrompe lo script in caso di errore
set -e

# Assicurati di essere nella root del workspace
cd "$(dirname "$0")"

echo "====================================="
echo "0. Indicizzazione di bme_gazebo_sensors..."
echo "====================================="

if [ ! -L "src/bme_gazebo_sensors" ] && [ ! -d "src/bme_gazebo_sensors" ]; then
    echo "Creo il link simbolico per bme_gazebo_sensors..."
    ln -s ../../bme_gazebo_sensors src/bme_gazebo_sensors
fi

echo "====================================="
echo "1. Sourcing di ROS 2 Jazzy..."
echo "====================================="
source /opt/ros/jazzy/setup.bash

echo "====================================="
echo "2. Compilazione del workspace..."
echo "====================================="
colcon build

echo "====================================="
echo "3. Sourcing del workspace locale..."
echo "====================================="
source install/setup.bash

echo "====================================="
echo "4. Avvio del file di launch..."
echo "====================================="
ros2 launch navigation_action_server navigation_launch.py
