#!/bin/bash
set -e

echo "Step 1: Updating System and Installing Essentials..."
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential cmake git libssl-dev libusb-1.0-0-dev pkg-config \
    libgtk-3-dev libglfw3-dev software-properties-common python3.12 python3.12-dev \
    python3.12-venv python3-pip

echo "Step 2: Installing NVIDIA Drivers and CUDA Toolkit..."
sudo apt install -y nvidia-driver-525 nvidia-cuda-toolkit
if command -v nvidia-smi &> /dev/null; then
    nvidia-smi
else
    echo "nvidia-smi command not found. Ensure NVIDIA drivers are installed correctly."
fi

echo "Step 3: Installing ROS 2 Jazzy..."
# Clean up existing ROS 2 repository entries more thoroughly
sudo rm -f /etc/apt/sources.list.d/ros2*.list
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 Jazzy repository (single entry)
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list

# Clean apt cache and update
sudo apt clean
sudo apt update

# Install ROS 2 Jazzy and build tools
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions

echo "Step 4: Setting up Python virtual environment..."
mkdir -p ~/Desktop/robot1
cd ~/Desktop/robot1

# Remove existing virtual environment if it exists
if [ -d ".venv" ]; then
    rm -rf .venv
fi

# Create fresh virtual environment
python3.12 -m venv .venv
source .venv/bin/activate

# Update bashrc, removing any existing entries first
sed -i '/source ~\/Desktop\/robot1\/.venv\/bin\/activate/d' ~/.bashrc
echo "# Activate RealSense project virtual environment" >> ~/.bashrc
echo "source ~/Desktop/robot1/.venv/bin/activate" >> ~/.bashrc

echo "Step 5: Building and Installing RealSense SDK from source..."
# Force remove existing librealsense directory with aggressive cleanup
echo "Cleaning up any existing librealsense installation..."
cd ~/Desktop/robot1
sudo chown -R $USER:$USER .
rm -rf librealsense
sudo rm -rf /usr/local/include/librealsense2
sudo rm -rf /usr/local/lib/librealsense2*
sudo rm -rf /usr/local/lib/cmake/realsense2

# Verify directory is gone and create fresh clone
echo "Cloning fresh copy of librealsense..."
if [ -d "librealsense" ]; then
    echo "ERROR: Could not remove librealsense directory. Please remove it manually with:"
    echo "sudo rm -rf ~/Desktop/robot1/librealsense"
    exit 1
fi

# Clone fresh copy
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
mkdir -p build && cd build
cmake .. -DBUILD_PYTHON_BINDINGS=ON -DPYTHON_EXECUTABLE=$(which python3.12)
make -j$(nproc)
sudo make install
cd ../..

echo "Step 6: Installing PyTorch, TorchVision, and TorchAudio..."
pip install --upgrade pip
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

echo "Step 7: Installing YOLO and other Python dependencies..."
pip install ultralytics opencv-python flask

echo "Step 8: Setting up ROS environment..."
# Clean up existing ROS source entries
sed -i '/source \/opt\/ros\/jazzy\/setup.bash/d' ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

echo "Step 9: Creating and building workspace..."
mkdir -p ~/Desktop/robot1/src
cd ~/Desktop/robot1
colcon build || echo "No packages found. Skipping build step."

echo "Step 10: Verifying installations..."
python -c "import pyrealsense2 as rs; print('RealSense pipeline available:', hasattr(rs, 'pipeline'))"
yolo predict model=yolov8n.pt source='https://ultralytics.com/images/bus.jpg'

echo "Setup complete! Please restart your terminal for all changes to take effect."
