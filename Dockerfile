FROM ros:kilted

RUN apt-get update && apt-get install -y wget xz-utils git git-lfs vim build-essential \
    ros-kilted-robot-localization ros-kilted-rviz2 ros-kilted-urdf ros-kilted-fuse ros-kilted-xacro \
    emacs vim nano gedit
RUN git lfs install
RUN wget https://go.microsoft.com/fwlink/?LinkID=760868 -O code.deb
RUN apt install ./code.deb -y
RUN rm code.deb
RUN mkdir /root/.code


WORKDIR /root/ws/src
RUN git clone https://github.com/locusrobotics/roscon-uk-2025-se-workshop.git
RUN git clone https://github.com/cra-ros-pkg/robot_localization.git -b kilted-devel
RUN cd roscon-uk-2025-se-workshop && git fetch origin answers:answers
WORKDIR /root/ws/src/roscon-uk-2025-se-workshop/bags
RUN ./decompress.sh
RUN rm *.xz decompress.sh

WORKDIR /root/ws
RUN  /bin/bash -c "source /opt/ros/kilted/setup.bash && cd /root/ws && colcon build --symlink-install --packages-skip task8"

ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ENV ws=/root/ws
ENV task1=/root/ws/src/roscon-uk-2025-se-workshop/task1
ENV task2=/root/ws/src/roscon-uk-2025-se-workshop/task2
ENV task3=/root/ws/src/roscon-uk-2025-se-workshop/task3
ENV task4=/root/ws/src/roscon-uk-2025-se-workshop/task4
ENV task5=/root/ws/src/roscon-uk-2025-se-workshop/task5
ENV task6=/root/ws/src/roscon-uk-2025-se-workshop/task6
ENV task7=/root/ws/src/roscon-uk-2025-se-workshop/task7
ENV task8=/root/ws/src/roscon-uk-2025-se-workshop/task8
ENV bags=/root/ws/src/roscon-uk-2025-se-workshop/bags

RUN echo "alias s='source /root/ws/install/setup.bash'" >> /root/.bashrc
RUN echo "alias cb='cd /root/ws && colcon build --symlink-install && cd -'" >> /root/.bashrc
RUN echo "alias vscode='code --no-sandbox --user-data-dir /root/.code -r /root/ws/src/roscon-uk-2025-se-workshop'" >> /root/.bashrc
RUN echo "s" >> /root/.bashrc
