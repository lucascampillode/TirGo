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
    groupadd -g 1000 ud && \
    useradd -ms /bin/bash ud -u 1000 -g 1000 && \
    echo "ud ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers && \
    echo "source /home/ud/carpeta_compartida/setup_env.sh" >> /home/ud/.bashrc && \
    RUN echo 'ud:ud' | chpasswd && \
    chown ud:ud /home/ud && \
    mkdir -p /var/run/ud/1000 && \
    chown ud:ud /var/run/ud/1000

USER ud

ENTRYPOINT [ "sleep", "infinity"]