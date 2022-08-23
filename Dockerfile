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

COPY DataStreamSDK_10.1 /ros_ws/DataStreamSDK_10.1
COPY install_libs.sh install_libs.sh
RUN ./install_libs.sh

# Copy the receiver accross and build
COPY vicon_receiver /ros_ws/src/vicon_receiver
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build\
    && rm -r build

EXPOSE 801

COPY run.sh run.sh
CMD [ "./run.sh" ]


