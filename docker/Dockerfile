FROM ros_humble_desktop_full_arm

ARG ROS_PACKAGE=ros_base \
    ROS_VERSION=humble

ENV ROS_DISTRO=${ROS_VERSION} \
    ROS_ROOT=/opt/ros/${ROS_VERSION} \
    ROS_PYTHON_VERSION=3 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    DEBIAN_FRONTEND=noninteractive \
    SHELL=/bin/bash

RUN apt-get update \
    && apt-get install -y \
    nano \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

RUN apt-get update \
    && apt-get install -y jstest-gtk\
    usbutils \
    xxd \
    net-tools \
    iputils-ping \
    fzf \
    python3-pip \
    wget \
    libgeographic-dev \
    mesa-utils \
    && rm -rf /ver/lib/apt/lists/*

# Download ROS2 packages
RUN apt-get update \
    && apt-get install -y \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui\
    ros-$ROS_DISTRO-tf-transformations \
    ros-$ROS_DISTRO-imu-tools \
    ros-$ROS_DISTRO-plotjuggler-ros \
    ros-$ROS_DISTRO-py-trees \
    ros-$ROS_DISTRO-py-trees-ros \
    ros-$ROS_DISTRO-nmea-msgs \
    ros-$ROS_DISTRO-point-cloud-transport

USER root

# install NVIDIA software
RUN apt-get update \
    && apt-get -y --quiet --no-install-recommends install \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    && rm -rf /var/lib/apt/lists/*

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1

# Install stonefish library

RUN apt-get update && apt-get install -y \
    python3 python3-pip python3-venv \
    git cmake build-essential \
    libopencv-dev libglm-dev libsdl2-dev libfreetype6-dev doxygen \
    libx11-dev x11-apps zstd \
    && rm -rf /var/lib/apt/lists/*

RUN sed -i 's/-lSDL2 /-lSDL2/' /usr/lib/*-linux-gnu/cmake/SDL2/sdl2-config.cmake

RUN git clone --branch agh-marines-branch https://github.com/AGH-Marines/stonefish.git  /home/$USERNAME/stonefish \
    && cd /home/$USERNAME/stonefish \
    && mkdir build \ 
    && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install \
    && export LD_LIBRARY_PATH=/home/$USERNAME/stonefish/build/libStonefish.so:$LD_LIBRARY_PATH

RUN mkdir -p /home/$USERNAME/ros2_ws
RUN chown -R ${USERNAME}:root /home/${USERNAME}/ros2_ws

# Clone packages for developement

## Dualshock 4 driver and package
RUN sudo pip3 install ds4drv
RUN sudo pip3 install osqp
RUN sudo pip3 install casadi
RUN sudo pip3 install pyserial

USER ${USERNAME}

USER root

COPY  . /home/$USERNAME/ros2_ws/

WORKDIR /home/$USERNAME/ros2_ws
RUN sudo apt install zstd

# Instalujemy wget i zstd
RUN sudo apt-get update && sudo apt-get install -y wget zstd

# Pobieramy właściwy instalator SDK bezpośrednio w Dockerze, uruchamiamy go i usuwamy plik instalacyjny (aby oszczędzić miejsce)
RUN wget https://download.stereolabs.com/zedsdk/5.1/l4t36.3/jetsons -O ZED_SDK_Tegra_L4T36.3_v5.3.0.zstd.run && \
    chmod +x ZED_SDK_Tegra_L4T36.3_v5.3.0.zstd.run && \
    ./ZED_SDK_Tegra_L4T36.3_v5.3.0.zstd.run -- silent && \
    rm ZED_SDK_Tegra_L4T36.3_v5.3.0.zstd.run

RUN git clone https://github.com/kimushun1101/ros2-aliases.git /home/dev/ros2-aliases
COPY .devcontainer/ros2aliases.env /home/dev/ros2-aliases/.env
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=${CUDA_HOME}/bin:${PATH}
ENV LD_LIBRARY_PATH=${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}
ENV CUDACXX=/usr/local/cuda/bin/nvcc

RUN rosdep install --from-paths src --ignore-src -r -y
#RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install && source /home/'$USERNAME'/ros2_ws/install/setup.bash"

#RUN #echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /home/$USERNAME/.bashrc \
#    && echo 'source /home/'$USERNAME'/ros2-aliases/ros2_aliases.bash' >> /home/$USERNAME/.bashrc \
#    && echo 'source /home/'$USERNAME'/ros2_ws/install/setup.bash' >> /home/$USERNAME/.bashrc

CMD ["bash"]