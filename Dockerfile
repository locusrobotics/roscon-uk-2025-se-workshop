FROM ros:kilted

RUN apt-get update && apt-get install -y wget xz-utils git vim build-essential \
    ros-kilted-robot-localization ros-kilted-rviz2
RUN rm -rf /var/lib/apt/lists/*

COPY task1 /root/ws/src/task1

WORKDIR /root/bags/planar
RUN wget https://rosconuk2025.s3.us-east-1.amazonaws.com/planar/planar.db3
RUN wget https://rosconuk2025.s3.us-east-1.amazonaws.com/planar/metadata.yaml

WORKDIR /root/ws
RUN  /bin/bash -c "source /opt/ros/kilted/setup.bash && cd /root/ws && colcon build --symlink-install"

ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

RUN echo "alias s='source install/setup.bash'" >> /root/.bashrc
RUN echo "alias cb='colcon build --symlink-install'" >> /root/.bashrc
RUN echo "source /root/ws/install/setup.bash" >> /root/.bashrc
