# Install Ubuntu 24.04 (ROS2 Jazzy) with user "developer", matching the
# StepIt robot's own container so the two can discover each other's ROS2
# graph over DDS when both run with --network host (see cyclonedds.xml).

FROM ros:jazzy-ros-base

LABEL name="develop:commander-ros2"

RUN apt-get update

RUN apt-get -y install sudo
RUN echo 'root:docker' | chpasswd                      # Update user root password.

# ros:jazzy-ros-base (Ubuntu 24.04) ships a default 'ubuntu' user at
# uid/gid 1000. Remove it so 'developer' can be created with a matching
# uid/gid to the host user (passed in via build args), which keeps the
# bind-mounted workspace writable.
RUN if id -u ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi
RUN if getent group ubuntu >/dev/null 2>&1; then groupdel ubuntu; fi

ARG USER_UID=1000
ARG USER_GID=1000

# Create the user developer with a uid/gid matching the host user so files
# created in the bind-mounted workspace (e.g. colcon's build/install/log
# directories) are writable and owned correctly on the host.
RUN groupadd -g ${USER_GID} developer
RUN useradd --create-home --shell /bin/bash --uid ${USER_UID} --gid ${USER_GID} developer
RUN echo 'developer:developer' | chpasswd              # Update user developer password.
RUN usermod -aG sudo developer                          # Add user to sudoers.

RUN apt-get -y install python3-pip
RUN apt-get -y install python3-colcon-common-extensions
RUN apt-get -y install python3-pytest
RUN apt-get install -y --no-install-recommends ros-jazzy-launch
RUN apt-get install -y --no-install-recommends ros-jazzy-launch-ros
RUN apt-get install -y --no-install-recommends ros-jazzy-rmw-cyclonedds-cpp
RUN apt-get install -y --no-install-recommends ros-jazzy-control-msgs
RUN apt-get install -y --no-install-recommends ros-jazzy-sensor-msgs
RUN apt-get install -y --no-install-recommends ros-jazzy-trajectory-msgs
RUN apt-get install -y --no-install-recommends ros-jazzy-controller-manager-msgs

# BehaviorTree.CPP and the dependencies of BehaviorTree.ROS2, which is not
# released as a Debian package and is therefore vendored into the workspace
# by ./bin/import.sh (see commander.repos).
RUN apt-get install -y --no-install-recommends ros-jazzy-behaviortree-cpp
RUN apt-get install -y --no-install-recommends ros-jazzy-generate-parameter-library
RUN apt-get install -y --no-install-recommends libboost-dev libfmt-dev libyaml-cpp-dev

# Testing and tooling.
RUN apt-get install -y --no-install-recommends ros-jazzy-ament-cmake-gmock
RUN apt-get install -y --no-install-recommends ros-jazzy-ament-cmake-gtest
RUN apt-get install -y --no-install-recommends git python3-vcstool

# Pre-commit hooks: the ament linters they run come from the ROS workspace, so
# they only work in a shell where /opt/ros/jazzy/setup.bash has been sourced,
# i.e. in this container. clang-format is the one hook that is not a ROS tool,
# and Ubuntu 24.04 ships the same major version (18) as the host, so the code
# is formatted the same way inside and outside the container.
RUN apt-get install -y --no-install-recommends pre-commit clang-format

# NOTE: the apt python3-websockets crashes on some ROS2 base images -- it
# still passes the removed `loop=` kwarg to asyncio.Lock() internally.
# Install a current version from PyPI instead.
RUN pip3 install --no-cache-dir --break-system-packages "websockets>=12"

CMD ["bash"]

USER developer

RUN mkdir -p /home/developer/.ros
COPY --chown=developer:developer cyclonedds.xml /home/developer/.ros

ENV HOME=/home/developer
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
RUN echo "export CYCLONEDDS_URI=~/.ros/cyclonedds.xml" >> ~/.bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

WORKDIR /home/developer/ws
