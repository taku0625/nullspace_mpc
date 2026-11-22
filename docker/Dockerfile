FROM osrf/ros:jazzy-desktop-full

COPY src /tmp/nullspace_mpc/src
RUN apt-get update \
    && rosdep update --rosdistro jazzy \
    && rosdep install -y --from-paths /tmp/nullspace_mpc/src --ignore-src --rosdistro jazzy \
    && rm -rf /tmp/nullspace_mpc /var/lib/apt/lists/*

WORKDIR /ws
