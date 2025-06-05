FROM osrf/ros:humble-desktop-jammy
EXPOSE 50002
ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=multirobots
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Workaround for the expired GPG key
RUN rm /etc/apt/sources.list.d/ros2-latest.list \
  && rm /usr/share/keyrings/ros2-latest-archive-keyring.gpg

RUN apt-get update \
  && apt-get install -y ca-certificates curl

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F \" '{print $4}') ;\
    curl -L -s -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb

# Install dependencies and tools
RUN apt-get update && apt-get install -y --fix-missing \
    git wget curl python3-pip nano net-tools terminator

# Install required packages
RUN apt-get update && apt-get install -y \
    x11-apps \
    rviz \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# Create the non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
    
# Modification of /etc/passwd to use /bin/bash as the default shell and get back auto-completion in terminator
RUN sed -i 's:/bin/sh:/bin/bash:' /etc/passwd 

# Change the owner of the home repository (recursively) : root -> multirobots
RUN chown -R $USERNAME:$USERNAME /home/$USERNAME
    
# WORKDIR // cd
WORKDIR /home/$USERNAME

# PX4 dependencies
RUN apt-get update && \
    git clone --recursive https://github.com/Adrien420/PX4-Autopilot.git && \
    cd PX4-Autopilot && \
    bash Tools/setup/ubuntu.sh && \
    cd .. && rm -rf PX4-Autopilot
    
# Micro-XRCE-DDS-Agent compilation
#COPY multirobots_ws/src/Micro-XRCE-DDS-Agent /home/$USERNAME/multirobots_ws/src/Micro-XRCE-DDS-Agent
#WORKDIR /home/$USERNAME/multirobots_ws/src
#RUN apt-get update && \
#    cd Micro-XRCE-DDS-Agent && \
#    mkdir -p build && \
#    cd build && \
#    cmake .. && \
#    make && \
#    sudo make install && \
#    sudo ldconfig
    
# ros_gz_sim & gz_ros2_contol dependencies

# We copy the packages ros_gz_sim & gz_ros2_contol (which will be erased by host's packages during the docker run), so that we can install their dependencies
COPY multirobots_ws/src/gz_ros2_control /home/$USERNAME/multirobots_ws/src/gz_ros2_control
COPY multirobots_ws/src/ros_gz/ros_gz_sim /home/$USERNAME/multirobots_ws/src/ros_gz/ros_gz_sim
WORKDIR /home/$USERNAME/multirobots_ws
ENV GZ_VERSION=harmonic
RUN rosdep update && \
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
	
# Set the default user
USER $USERNAME
WORKDIR /home/$USERNAME
    
# Source ROS setup file
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
RUN echo "source ~/multirobots_ws/install/setup.bash" >> ~/.bashrc
