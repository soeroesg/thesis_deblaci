#!/bin/bash

# This is a script for installing the dependencies that are required
# for my thesis project

read -p 'Please specify your desired workspace: ' WORKSPACE
cd $WORKSPACE

sudo apt install git python-is-python3 python3-pip git ffmpeg -y
git clone git@github.com:SpectacularAI/sdk-examples.git
pip install spectacularAI[full]

sudo apt install ros-humble-rtabmap-ros ros-humble-depthai-ros -y

