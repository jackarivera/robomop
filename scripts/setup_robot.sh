#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Navigate to the workspace
cd ~/robomop_ws/robomop

# Update and upgrade the system
sudo apt update && sudo apt upgrade -y

# Run the dependencies installation script
./scripts/setup_dependencies.sh

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash

echo "Robomop setup complete!"
