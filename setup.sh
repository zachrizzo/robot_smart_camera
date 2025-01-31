#!/bin/bash
set -e

echo "Setting up RealSense Robot AI environment..."

# Install Poetry if not already installed
if ! command -v poetry &> /dev/null; then
    echo "Installing Poetry..."
    curl -sSL https://install.python-poetry.org | python3 -
fi

# Configure Poetry to use the active Python environment
poetry config virtualenvs.prefer-active-python true

# Install Python dependencies using Poetry
echo "Installing Python dependencies with Poetry..."
poetry install

# Install ROS2 system dependencies
echo "Installing ROS2 system dependencies..."
sudo apt-get update
sudo apt-get install -y \
    python3-rclpy \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    python3-opencv \
    python3-numpy

# Install Ollama (if not already installed)
if ! command -v ollama &> /dev/null; then
    echo "Installing Ollama..."
    curl -fsSL https://ollama.com/install.sh | sh
fi

# Start Ollama service
echo "Starting Ollama service..."
ollama serve &

# Pull required models
echo "Pulling required Ollama models..."
ollama pull llama2

# Build ROS2 package
echo "Building ROS2 package..."
colcon build --symlink-install

echo "Setup complete! Please:"
echo "1. Add your API keys to the .env file"
echo "2. Activate the Poetry environment:"
echo "   poetry shell"
echo "3. Source the ROS2 workspace:"
echo "   source install/setup.bash"
echo "4. Run the launch file:"
echo "   ros2 launch realsense_robot robot.launch.py"
