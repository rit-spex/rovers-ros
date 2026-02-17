#!/bin/bash
# Stop script on error
set -e

echo "Starting install for JetPack 6.1 (Ubuntu 22.04 / CUDA 12.6)..."

# 1. Install System Dependencies
sudo apt-get update
sudo apt-get install -y libopenblas-base libopenmpi-dev libomp-dev \
libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev

# 2. Install PyTorch (Specific Wheel for JetPack 6.1 / CUDA 12.6)
# URL updated to 'v61' folder
wget https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl -O torch.whl
pip3 install torch.whl

# 3. Install TorchVision (Compile from source)
# We use v0.20.0 to match PyTorch 2.5
echo "Building TorchVision... this will take 15-20 minutes."
pip3 uninstall -y torchvision
git clone https://github.com/pytorch/vision torchvision
cd torchvision
git checkout v0.20.0 
export BUILD_VERSION=0.20.0
python3 setup.py install --user

echo "Installation Complete. Please restart your terminal."