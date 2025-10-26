# Dockerfile for Argo Shore Station (Windows/Mac/Linux)
# 
# This provides a containerized environment for running:
#   - lora_shore.py (LoRa receiver)
#   - argo_web_dashboard.py (Web dashboard)
# 
# Build: docker build -t argo-shore .
# Run:   docker run --rm -it --privileged -v /dev:/dev -p 8081:8081 argo-shore

FROM osrf/ros:humble-desktop

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install pyserial flask flask-cors

# Create working directory
WORKDIR /argo

# Copy Argo files (user will mount these)
# COPY shore/ /argo/shore/
# COPY nodes/ /argo/nodes/
# COPY launch/ /argo/launch/

# Expose web dashboard port
EXPOSE 8081

# Source ROS2 and set up environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Default command: run both services
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && python3 shore/lora_shore.py --port /dev/ttyUSB0 & sleep 5 && python3 nodes/argo_web_dashboard.py"]

