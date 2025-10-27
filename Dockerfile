FROM ros:noetic

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# =========================
#  Dependencias de sistema (añadimos audio)
# =========================
RUN apt update && apt install -y --no-install-recommends \
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
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-teleop-twist-joy \
    # ---- AUDIO ----
    alsa-utils libasound2 libasound2-data libasound2-dev \
    portaudio19-dev python3-pyaudio \
 && rm -rf /var/lib/apt/lists/*

# =========================
#  Librerías Python (añadimos vosk + sounddevice)
# =========================
RUN pip install --no-cache-dir \
    catkin_tools \
    tensorboard \
    opencv-python \
    flask \
    flask-cors \
    vosk \
    pyaudio \
    sounddevice \
    numpy

# =========================
#  Crear usuario TirGo
# =========================
RUN groupadd -g 1000 TirGo && \
    useradd -ms /bin/bash TirGo -u 1000 -g 1000 && \
    echo "TirGo ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers && \
    echo "source /opt/ros/noetic/setup.bash" >> /home/TirGo/.bashrc && \
    echo "source /home/TirGo/carpeta_compartida/setup_env.sh" >> /home/TirGo/.bashrc && \
    echo 'TirGo:TirGo' | chpasswd && \
    chown -R TirGo:TirGo /home/TirGo && \
    mkdir -p /var/run/TirGo/1000 && \
    chown TirGo:TirGo /var/run/TirGo/1000 && \
    usermod -aG audio TirGo

# =========================
#  ALSA por defecto → tu micro hw:1,7 a 16 kHz mono
# =========================
RUN printf '%s\n' \
'pcm.!default {' \
'  type plug' \
'  slave {' \
'    pcm "hw:1,7"' \
'    rate 16000' \
'    channels 1' \
'  }' \
'}' > /home/TirGo/.asoundrc && chown TirGo:TirGo /home/TirGo/.asoundrc



ENTRYPOINT ["sleep", "infinity"]
