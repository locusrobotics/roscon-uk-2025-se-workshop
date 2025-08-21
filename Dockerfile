FROM ros:kilted

RUN apt-get update && apt-get install -y wget xz-utils git git-lfs vim build-essential \
    ros-kilted-robot-localization ros-kilted-rviz2
RUN git lfs install
RUN rm -rf /var/lib/apt/lists/*

WORKDIR /root/ws/src
RUN git clone https://github.com/ayrton04/roscon-uk-2025-se-workshop.git
WORKDIR /root/ws/src/roscon-uk-2025-se-workshop/bags
RUN ./decompress.sh
RUN rm *.xz decompress.sh

WORKDIR /root/ws
RUN  /bin/bash -c "source /opt/ros/kilted/setup.bash && cd /root/ws && colcon build --symlink-install"

ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

RUN echo "alias s='source install/setup.bash'" >> /root/.bashrc
RUN echo "alias cb='colcon build --symlink-install'" >> /root/.bashrc
RUN echo "source /root/ws/install/setup.bash" >> /root/.bashrc
