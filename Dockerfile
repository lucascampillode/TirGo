FROM ros:noetic

# =========================
#  Instalar dependencias
# =========================
RUN apt update && apt install -y \
    rviz \
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
    libxmlrpc-core-c3-dev \
 && rm -rf /var/lib/apt/lists/*

# =========================
#  Librerías Python
# =========================
RUN pip install --no-cache-dir \
    catkin_tools \
    tensorboard \
    opencv-python \
    flask \
    flask-cors

# =========================
#  Crear usuario TirGo
# =========================
RUN groupadd -g 1000 TirGo && \
    useradd -ms /bin/bash TirGo -u 1000 -g 1000 && \
    echo "TirGo ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers && \
    echo "source /home/TirGo/carpeta_compartida/setup_env.sh" >> /home/TirGo/.bashrc && \
    echo 'TirGo:TirGo' | chpasswd && \
    chown -R TirGo:TirGo /home/TirGo && \
    mkdir -p /var/run/TirGo/1000 && \
    chown TirGo:TirGo /var/run/TirGo/1000

USER TirGo

ENTRYPOINT ["sleep", "infinity"]
