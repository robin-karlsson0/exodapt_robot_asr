FROM ros:humble

# Set up a workspace
WORKDIR /ros2_ws

# Copy your custom packages
COPY ./ros2_ws/src ./ros2_ws/

# Install dependencies
RUN apt-get update && apt-get install -y \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Build your packages
WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build

# Set up the environment
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

COPY ./run_asr_riva_bridge.sh ./run_asr_riva_bridge.sh

# Set the entrypoint
# ENTRYPOINT ["/bin/bash"]
# CMD [". /ros2_ws/install/setup.bash"]
