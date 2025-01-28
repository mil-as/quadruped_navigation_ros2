ARG BASE_IMAGE=nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_3a3f78e549dd2b911006ee1756cd5525
FROM ${BASE_IMAGE}

ARG USERNAME=isaac_ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Check if group exists, if not, add it
    && (grep -q "^[^:]*:[^:]*:${USER_GID}:" /etc/group || groupadd --gid ${USER_GID} ${USERNAME}) \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && mkdir -p /home/${USERNAME}/.config \
    && chown ${USER_UID}:${USER_GID} /home/${USERNAME}/.config \
    && rm -rf /var/lib/apt/lists/*

# Sett arbeidskatalogen til brukerens hjem
WORKDIR /home/${USERNAME}

# Fra https://dev.to/koheikawata/devcontainer-for-ros2-project-31ng
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/isaac_ros/.bashrc

USER ${USERNAME}