# 1. The Base Environment
FROM ros:humble-ros-base

# 2. System Tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    libgl1-mesa-glx \
    libglib2.0-0 \
    ros-humble-cv-bridge \
    ros-humble-v4l2-camera \
    && rm -rf /var/lib/apt/lists/*

# 3. Python Libraries
RUN pip3 install \
    opencv-python \
    ultralytics \
    numpy

# 4. The Workspace (container opens directly into that directory)
WORKDIR /app
