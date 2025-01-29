ARG BASE_IMAGE=nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_3a3f78e549dd2b911006ee1756cd5525
FROM ${BASE_IMAGE}

ARG USERNAME=isaac_ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# Installs packedes
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    ros-humble-pcl-conversions

# Builds Livox SDK2
#RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git \
#    && cd Livox-SDK2 \
#    && mkdir build \
#    && cd build \
#    && cmake .. \
#    && make -j \
#    && sudo make install

# Check if group exists, if not, add it
RUN (grep -q "^[^:]*:[^:]*:${USER_GID}:" /etc/group || groupadd --gid ${USER_GID} ${USERNAME}) \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && mkdir -p /home/${USERNAME}/.config \
    && chown ${USER_UID}:${USER_GID} /home/${USERNAME}/.config \
    && rm -rf /var/lib/apt/lists/*

# Adds sudo to user
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && rm -rf /var/lib/apt/lists/*

# Setts the user workspace
WORKDIR /home/${USERNAME}

# Fra https://dev.to/koheikawata/devcontainer-for-ros2-project-31ng
# Source ROS
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/isaac_ros/.bashrc

USER ${USERNAME}