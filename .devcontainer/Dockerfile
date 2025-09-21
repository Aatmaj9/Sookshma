# Base: Ubuntu 22.04 + ROS Humble
FROM ros:humble

RUN apt-get update
RUN apt-get -y install software-properties-common
RUN add-apt-repository universe
RUN apt-get update
RUN apt-get -y install python3-pip python3-serial
RUN apt-get -y install sudo xauth htop udev zip unzip curl
RUN apt-get update --fix-missing
RUN apt-get -y install libboost-all-dev libasio-dev 

# --- System deps & ROS 2 packages in one layer ---
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential gcc gfortran git \
      python3-pip python3-serial python3-venv \
      libsm6 libxext6 libxrender1 libfontconfig1 libgl1 libboost-all-dev libasio-dev \
      gdebi-core \
      ros-humble-diagnostic-updater \
      ros-humble-nmea-msgs \
      ros-humble-rtcm-msgs \
      ros-humble-rosbridge-server \
      ros-humble-image-transport-plugins \
      ros-humble-compressed-image-transport \
      ros-humble-compressed-depth-image-transport \
      ros-humble-theora-image-transport \
      ros-humble-point-cloud-transport-plugins \
      ros-humble-draco-point-cloud-transport \
      ros-humble-zlib-point-cloud-transport \
      ros-humble-zstd-point-cloud-transport \
      ros-humble-xacro \
      ros-humble-ffmpeg-image-transport \
      ros-humble-ffmpeg-encoder-decoder \
      ros-humble-zed-msgs \
      ros-humble-geographic-msgs \
      ros-humble-image-transport \
      ros-humble-point-cloud-transport \
      ros-humble-robot-localization \
      ros-humble-backward-ros \
    && ln -sf /usr/bin/python3 /usr/bin/python \
    && rm -rf /var/lib/apt/lists/*

# --- arm64-only: system toolchain/uploader (Due needs bossac) ---
ARG TARGETARCH
RUN if [ "$TARGETARCH" = "arm64" ]; then \
      apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi bossa-cli && \
      rm -rf /var/lib/apt/lists/*; \
    fi
    
#-----------------------------------------------------------------------------------All basic installations is done---------------------------------------------------------------------------

# --- Create non-root user once (sudo + dialout) ---
ARG USERNAME=mavlab
ENV USERNAME=${USERNAME}
RUN useradd -m "${USERNAME}" \
    && echo "${USERNAME}:${USERNAME}" | chpasswd \
    && usermod --shell /bin/bash "${USERNAME}" \
    && usermod -aG sudo,dialout "${USERNAME}" \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}

#---------------------------------------------------------------------------------Python Dependencies and  Arduino CLI---------------------------------------------------------------------

# --- Python deps from requirements.txt -----
COPY --chown=${USERNAME}:${USERNAME} requirements.txt /home/${USERNAME}/requirements.txt
RUN pip install --user -r /home/${USERNAME}/requirements.txt
ENV PATH="/home/${USERNAME}/.local/bin:${PATH}"

# --- Arduino CLI Setup---
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
ENV PATH="/home/${USERNAME}/bin:${PATH}"
ENV ARDUINO_DATA="/home/${USERNAME}/.arduino15"
RUN arduino-cli config init \
 && arduino-cli config set directories.data "${ARDUINO_DATA}"
RUN arduino-cli core update-index && arduino-cli cache clean

# ----------------------------------------------------------------------------------Arduino Installations on AMD ----------------------------------------------------------------------------------

# This RUN block only executes if the target architecture is 'AMD64'
ARG TARGETARCH
RUN if [ "$TARGETARCH" = "amd64" ]; then \
        echo "--- Executing AMD-specific Arduino setup ---" && \
        # Install the standard Arduino SAM core
        arduino-cli core update-index && \
        arduino-cli cache clean && \
        arduino-cli core install arduino:sam@1.6.12 && \
        # Define a shell variable for the path
        SAM_AMD_PATH="/home/${USERNAME}/.arduino15/packages/arduino/hardware/sam/1.6.12" && \
        # Patch the platform.txt file
        curl -fsSL https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/main/extras/patching_boards/platform_arduinocore_sam.txt -o ${SAM_AMD_PATH}/platform.txt && \
        # Install libraries
        arduino-cli lib install Servo && \
        git clone --depth=1 https://gitlab.com/timwilkinson/FlySkyIBus.git /home/${USERNAME}/Arduino/libraries/FlySkyIBus; \
    fi

COPY --chown=${USERNAME}:${USERNAME} micro_ros_arduino_amd.zip /tmp/micro_ros_arduino.zip

#Install the pre-built custom micro_ros_arduino for AMD
RUN if [ "$TARGETARCH" = "amd64" ]; then \
        echo "--- Executing AMD-specific user setup ---" && \
        # Create arduino-cli config
        printf "board_manager:\n  additional_urls: []\n\nlibrary:\n  enable_unsafe_install: true\n" > /home/${USERNAME}/.arduino15/arduino-cli.yaml && \
        # Install the zipped micro-ROS library
        arduino-cli --config-file /home/${USERNAME}/.arduino15/arduino-cli.yaml lib install --zip-path /tmp/micro_ros_arduino.zip && \
        rm /tmp/micro_ros_arduino.zip; \
    fi
# -----------------------------------------------------------------------------------Arduino Installations on ARM----------------------------------------------------------------------------------

# This RUN block only executes if the target architecture is 'ARM64'
ARG TARGETARCH
RUN if [ "$TARGETARCH" = "arm64" ]; then \
        echo "--- Executing ARM-specific Arduino setup ---" && \
        # Install and configure the core
        arduino-cli core update-index && \
        arduino-cli cache clean && \
        arduino-cli core update-index --additional-urls https://per1234.github.io/ArduinoCore-sam/package_per1234_samarm64_index.json && \
        export ARDUINO_BOARD_MANAGER_ADDITIONAL_URLS=https://per1234.github.io/ArduinoCore-sam/package_per1234_samarm64_index.json && \
        arduino-cli core install per1234:sam && \
        # Define a shell variable for the path
        SAM_ARM_PATH="/home/${USERNAME}/.arduino15/packages/per1234/hardware/sam/1.6.11-arm64" && \
        # Patch the platform.txt file
        curl -fsSL https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/main/extras/patching_boards/platform_arduinocore_sam.txt -o ${SAM_ARM_PATH}/platform.txt && \
        sed -i 's/-4.8.3-2014q1././g' ${SAM_ARM_PATH}/platform.txt && \
        # Install basic libraries
        arduino-cli lib install Servo && \
        git clone --depth=1 https://gitlab.com/timwilkinson/FlySkyIBus.git /home/${USERNAME}/Arduino/libraries/FlySkyIBus; \
    fi

COPY --chown=${USERNAME}:${USERNAME} micro_ros_arduino_arm.zip /tmp/micro_ros_arduino.zip

#Install the pre-built custom micro_ros_arduino for ARM
RUN if [ "$TARGETARCH" = "arm64" ]; then \
        echo "--- Executing ARM-specific user setup ---" && \
        # Create arduino-cli config
        printf "board_manager:\n  additional_urls: []\n\nlibrary:\n  enable_unsafe_install: true\n" > /home/${USERNAME}/.arduino15/arduino-cli.yaml && \
        # Install the zipped micro-ROS library
        arduino-cli --config-file /home/${USERNAME}/.arduino15/arduino-cli.yaml lib install --zip-path /tmp/micro_ros_arduino.zip && \
        rm /tmp/micro_ros_arduino.zip; \
    fi

# --------------------------------------------------------------------------------------Some Other Installations--------------------------------------------------------------------------------

# --- Node via NVM (user-local) + localtunnel ---
ENV NVM_DIR="/home/${USERNAME}/.nvm"    
ENV NODE_VERSION="20.17.0"
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | bash \
 && . "$NVM_DIR/nvm.sh" \
 && nvm install $NODE_VERSION \
 && nvm alias default $NODE_VERSION \
 && nvm use default \
 && npm install -g npm@latest localtunnel
ENV PATH="${NVM_DIR}/versions/node/v${NODE_VERSION}/bin:${PATH}"

#--------------------------------------------------------------------------------------------ROS-Convinience------------------------------------------------------------------------------------

RUN echo 'source /opt/ros/humble/setup.bash' >> /home/${USERNAME}/.bashrc \
 && echo 'if [ -f /workspaces/mavlab/ros2_ws/install/setup.bash ]; then source /workspaces/mavlab/ros2_ws/install/setup.bash; fi' >> /home/${USERNAME}/.bashrc \
 && echo "alias sros2='source /opt/ros/humble/setup.bash && if [ -f /workspaces/mavlab/ros2_ws/install/setup.bash ]; then source /workspaces/mavlab/ros2_ws/install/setup.bash; fi'" >> /home/${USERNAME}/.bashrc \
 && echo "alias sbg='sros2 && ros2 launch sbg_driver sbg_device_launch.py'" >> /home/${USERNAME}/.bashrc \
 && echo "alias uwb='sros2 && ros2 run uwb_driver uwb'" >> /home/${USERNAME}/.bashrc \
 && echo "alias gnc='sros2 && ros2 run gnc gnc'" >> /home/${USERNAME}/.bashrc \
 && echo "alias mavsim='sros2 && ros2 run mav_simulator simulate'" >> /home/${USERNAME}/.bashrc \
 && echo "alias ardusimple='sros2 && ros2 launch ublox_gps ublox_gps_node-launch.py'" >> /home/${USERNAME}/.bashrc

# (Optional) Ports
EXPOSE 9000 9001 9002 9003
