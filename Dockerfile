FROM ros:foxy-ros-base-focal

RUN apt-get update \
    && apt-get install -y \
    curl gnupg2 lsb-release \
    python3-colcon-common-extensions \
    # libboost-dev \
    libboost-thread-dev \
    cmake \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros_ws

# Install DataStreamSDK
COPY DataStreamSDK_1.11/* /usr/lib/
RUN chmod -R 0755 /usr/lib \
    && export VICON_LIBRARY=/usr/lib \
    && ldconfig

# Copy the receiver accross and build
COPY vicon_receiver /ros_ws/src/vicon_receiver
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build\
    && rm -r build

EXPOSE 801

COPY run.sh run.sh
CMD [ "./run.sh" ]


