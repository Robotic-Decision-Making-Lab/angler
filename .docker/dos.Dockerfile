ARG ROS_DISTRO=rolling
FROM ros:$ROS_DISTRO-ros-base as ci

LABEL maintainer="Evan Palmer"
LABEL maintainer-email="evanp922@gmail.com"

ENV DEBIAN_FRONTEND=noninteractive

# Install apt packages
RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && apt-get -q install --no-install-recommends -y \
    git \
    wget \
    curl \
    sudo \
    clang \
    clang-format-14 \
    clang-tidy \
    clang-tools \
    python3-pip \
    python3-dev \
    software-properties-common \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Configure a new non-root user
ARG USERNAME=angler
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

# Switch to the non-root user to install MAVROS dependencies
USER $USERNAME
ENV USER=$USERNAME

# Install MAVROS dependencies
WORKDIR /home/$USERNAME
RUN wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && sudo ./install_geographiclib_datasets.sh

# Switch back to root user
USER root
ENV USER=root

# Install all ROS dependencies for testing
# Remove the source code from this stage
WORKDIR /root/ws_angler
COPY . src/angler

RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && rosdep update \
    && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false --skip-keys="gz-transport12 gz-sim7 gz-math7 gz-msgs9 gz-plugin2" \
    && rm -rf src \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

FROM ci as deps

ENV DEBIAN_FRONTEND=noninteractive
ENV GZ_VERSION=garden

# Install gstreamer
RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && apt-get -q install --no-install-recommends -y \
    python3-gi \
    gstreamer1.0-tools \
    gir1.2-gstreamer-1.0 \
    gir1.2-gst-plugins-base-1.0 \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Install the Python requirements that aren't available as rosdeps
COPY requirements-build.txt .
RUN python3 -m pip install -r requirements-build.txt \
    && rm -rf requirements-build.txt

# Switch to the non-root user
# We are going to use this for the rest of the installation
USER $USERNAME
ENV USER=$USERNAME

# Create a user-level ROS workspace for us to use
ENV USER_WORKSPACE=/home/$USERNAME/ws_angler
WORKDIR $USER_WORKSPACE

# Install the external project requirements
COPY --chown=$USER_UID:$USER_GID angler.repos src/
RUN sudo apt-get -q update \
    && sudo apt-get -q -y upgrade \
    && vcs import src < src/angler.repos \
    && rosdep update \
    && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --skip-keys="gz-transport12 gz-sim7 gz-math7 gz-msgs9 gz-plugin2" \
    && rm -rf angler.repos \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && colcon build \
    # Update .bashrc to source the workspace
    && echo "source ${USER_WORKSPACE}/install/setup.sh" >> /home/$USERNAME/.bashrc

# Now install any remaining Angler rosdeps
COPY --chown=$USER_UID:$USER_GID . src/angler
RUN sudo apt-get -q update \
    && sudo apt-get -q -y upgrade \
    && rosdep update \
    && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --skip-keys="gz-transport12 gz-sim7 gz-math7 gz-msgs9 gz-plugin2" \
    && rm -rf src/angler \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

# Install development tools for testing from the command line
# We install this here so that we can use it in the source and develop stages
RUN sudo apt-get -q update \
    && sudo apt-get -q -y upgrade \
    && sudo apt-get -q install --no-install-recommends -y \
    iputils-ping \
    net-tools \
    gdb \
    nano \
    htop \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

# Inspiration for this stage comes from the following source
# https://github.com/clydemcqueen/orca4/blob/77152829e1d65781717ca55379c229145d6006e9/docker/Dockerfile#L1

# Install Gazebo Garden: https://gazebosim.org/docs/garden/install_ubuntu
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN sudo apt-get update \
    && sudo apt-get -y --quiet --no-install-recommends install \
    gz-garden \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

# Install ArduPilot and ardupilot_gazebo dependencies
RUN sudo apt-get -q update \
    && sudo apt-get -q -y upgrade \
    && sudo apt-get -q install --no-install-recommends -y \
    python3-wxgtk4.0 \
    rapidjson-dev \
    xterm \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

# Clone ArduSub
# ArduSub is installed for simulation purposes ONLY
# When deployed onto hardware, the native installation of ArduSub
# (on the FCU) will be used.
WORKDIR /home/$USERNAME
RUN git clone https://github.com/ArduPilot/ardupilot.git --recurse-submodules

# Install ArduSub dependencies
WORKDIR /home/$USERNAME/ardupilot
ENV SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1
RUN Tools/environment_install/install-prereqs-ubuntu.sh -y

# Build ArduSub
WORKDIR /home/$USERNAME/ardupilot
RUN modules/waf/waf-light configure --board sitl \
    && modules/waf/waf-light build --target bin/ardusub

# Clone ardupilot_gazebo code
WORKDIR /home/$USERNAME
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git

# Install ardupilot_gazebo plugin
RUN [ "/bin/bash" , "-c" , " \
    cd ardupilot_gazebo \
    && mkdir build \
    && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && make -j4" ]

# Setup the environment variables needed for simulation
COPY .docker/entrypoints/sim.sh /
RUN echo "if [ -f /sim.sh ]; then source /sim.sh; fi" >> /home/$USERNAME/.bashrc

FROM deps as robot

ENV DEBIAN_FRONTEND=noninteractive

ENV USER_WORKSPACE=/home/$USERNAME/ws_angler
WORKDIR $USER_WORKSPACE

# Get the source code and build
# We don't need to update the .bashrc file this time, that was
# done in the previous stage
COPY --chown=$USER_UID:$USER_GID . src/angler
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && colcon build \
    && . "install/setup.sh"

FROM sim as develop

ENV DEBIAN_FRONTEND=noninteractive

# Install debugging/linting Python packages
WORKDIR /home/$USERNAME
COPY requirements-dev.txt .
RUN python3 -m pip install -r requirements-dev.txt \
    && rm -rf requirements-dev.txt

# Set up the development environment
COPY .docker/entrypoints/dev.sh /
RUN echo "if [ -f /dev.sh ]; then source /dev.sh; fi" >> /home/$USERNAME/.bashrc

# WARNING: This is a temporary solution for disabling the setuputils installation warning
ENV PYTHONWARNINGS="ignore"

ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

FROM sim as develop-nvidia

ENV DEBIAN_FRONTEND=noninteractive

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM 1

# Install debugging/linting Python packages
WORKDIR /home/$USERNAME
COPY requirements-dev.txt .
RUN python3 -m pip install -r requirements-dev.txt \
    && rm -rf requirements-dev.txt

# Set up the development environment
COPY .docker/entrypoints/dev.sh /
RUN echo "if [ -f /dev.sh ]; then source /dev.sh; fi" >> /home/$USERNAME/.bashrc

# WARNING: This is a temporary solution for disabling the setuputils installation warning
ENV PYTHONWARNINGS="ignore"

ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc
