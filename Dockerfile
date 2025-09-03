FROM ros:kilted

RUN apt-get update && apt-get install -y wget xz-utils git git-lfs vim build-essential \
    ros-kilted-robot-localization ros-kilted-rviz2 ros-kilted-urdf ros-kilted-fuse ros-kilted-xacro
RUN git lfs install
RUN rm -rf /var/lib/apt/lists/*

WORKDIR /root/ws/src
RUN git clone https://github.com/ayrton04/roscon-uk-2025-se-workshop.git
RUN cd roscon-uk-2025-se-workshop && git fetch origin answers:answers
WORKDIR /root/ws/src/roscon-uk-2025-se-workshop/bags
RUN ./decompress.sh
RUN rm *.xz decompress.sh

WORKDIR /root/ws
RUN  /bin/bash -c "source /opt/ros/kilted/setup.bash && cd /root/ws && colcon build --symlink-install --packages-select task1 task2 task3 task4 task5 task6 task7 task8"

ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ENV ws=/root/ws
ENV task1=/root/ws/src/roscon-uk-2025-se-workshop/task1
ENV task1=/root/ws/src/roscon-uk-2025-se-workshop/task2
ENV task3=/root/ws/src/roscon-uk-2025-se-workshop/task3
ENV task4=/root/ws/src/roscon-uk-2025-se-workshop/task4
ENV task5=/root/ws/src/roscon-uk-2025-se-workshop/task5
ENV task6=/root/ws/src/roscon-uk-2025-se-workshop/task6
ENV task7=/root/ws/src/roscon-uk-2025-se-workshop/task7
ENV task8=/root/ws/src/roscon-uk-2025-se-workshop/task8
ENV bags=/root/ws/src/roscon-uk-2025-se-workshop/bags

RUN echo "alias s='source /root/ws/install/setup.bash'" >> /root/.bashrc
RUN echo "alias cb='cd /root/ws && colcon build --symlink-install && cd -'" >> /root/.bashrc
RUN echo "s" >> /root/.bashrc
