FROM ros:noetic

RUN apt update && apt install -y rviz \
         ros-noetic-moveit \
         curl \
         apt-utils \
         python3-pip \
         python-is-python3 \
         ssh \
         net-tools \
         sudo \
         iputils-ping \
         nmap \
         xauth \
         ros-noetic-robot-state-publisher \
         ros-noetic-joint-state-publisher \
         git \
         ros-noetic-usb-cam \
         htop \
         sudo \
         libxmlrpc-core-c3-dev

RUN pip install catkin_tools \
                tensorboard \
                opencv-python

RUN rm -rf /var/lib/apt/lists/* && \
    groupadd -g 1000 TirGo && \
    useradd -ms /bin/bash TirGo -u 1000 -g 1000 && \
    echo "TirGo ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers && \
    echo "source /home/TirGo/carpeta_compartida/setup_env.sh" >> /home/TirGo/.bashrc && \
    RUN echo 'TirGo:TirGo' | chpasswd && \
    chown TirGo:TirGo /home/TirGo && \
    mkdir -p /var/run/TirGo/1000 && \
    chown TirGo:TirGo /var/run/TirGo/1000

USER TirGo

ENTRYPOINT [ "sleep", "infinity"]