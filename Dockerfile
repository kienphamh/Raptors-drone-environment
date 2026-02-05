FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    git \
    wget \
    vim \
    nano \
    curl \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# X11 and GUI
RUN apt-get update && apt-get install -y \
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Gazebo
RUN apt-get update && apt-get install -y \
    gz-harmonic \
    ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*


# MAVROS
RUN apt-get update && apt-get install -y \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
    && rm -rf /var/lib/apt/lists/*


# Install GeographicLib datasets for MAVROS
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && ./install_geographiclib_datasets.sh \
    && rm install_geographiclib_datasets.sh

# ArduPilot dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    python3-opencv \
    python3-wxgtk4.0 \
    python3-matplotlib \
    python3-lxml \
    python3-pygame \
    libxml2-dev \
    libxslt1-dev \
    && rm -rf /var/lib/apt/lists/*

# Mavproxy
RUN pip3 install --break-system-packages \
    MAVProxy \
    pymavlink

# Ardupilot and its environment
WORKDIR /opt
RUN git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git \
    && cd ardupilot \
    && git checkout Copter-4.5 \
    && Tools/environment_install/install-prereqs-ubuntu.sh -y
ENV PATH="/opt/ardupilot/Tools/autotest:${PATH}"
ENV PATH="/usr/lib/ccache:${PATH}"

RUN mkdir -p /workspace
WORKDIR /workspace

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

COPY aliases/ /root/aliases/
RUN chmod +x /root/aliases/*.sh && \
    echo "# Load drone aliases" >> ~/.bashrc && \
    echo "for f in /root/aliases/*.sh; do source \$f; done" >> ~/.bashrc

# Set display for X11
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/jazzy/setup.bash\n\
if [ -f /workspace/install/setup.bash ]; then\n\
    source /workspace/install/setup.bash\n\
fi\n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]