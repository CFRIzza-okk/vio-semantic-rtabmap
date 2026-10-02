FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
COPY sources.list /tmp/host-sources.list
COPY ros.gpg /etc/apt/trusted.gpg.d/ros.gpg
RUN cp /tmp/host-sources.list /etc/apt/sources.list.d/ros2.list && \
    rm -f /etc/apt/sources.list.d/ros2.sources && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rtabmap-ros \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    && rm -rf /var/lib/apt/lists/*

COPY launch_d435_rtabmap.sh /usr/local/bin/launch_d435_rtabmap
RUN chmod +x /usr/local/bin/launch_d435_rtabmap
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
