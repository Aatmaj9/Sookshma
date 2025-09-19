# SOOKSHMA REPO

## About the files

The build.sh script is to be used only for primary building the image on the system. The build.sh script will build multiplatform image - for amd64 and arm64. On all the RPIs use pull.sh  to pull the respective arm64 image.

Use the start.sh to start the container and the enter.sh to enter the container in another terminal.

libgen_arm.sh used only on the main system to generate the custom message compatible micro_ros_arduino library for ARM platforms. The library has been already built and provided as zip file which has already been installed in the dockerfile.

libgen_amd.sh used only on the main system to generate the custom message compatible micro_ros_arduino library for AMD platforms. The library has been already built and provided as zip file which has already been installed in the dockerfile.

The compile and upload scripts from inside the container to arduino from arm and amd systems respectively.

Do pull.sh to pull the latest image.

## Requirements for building the multi-platform image on the system

```
docker buildx create --name multiarch --driver docker-container --use 
docker run --privileged --rm tonistiigi/binfmt --install all 
docker buildx inspect --bootstrap

```
## Requirements for libgen on the system

Arduino

```
cd ~
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

echo 'export PATH=$PATH:$HOME/bin' >> ~/.bashrc
source ~/.bashrc

arduino-cli core update-index
arduino-cli cache clean
arduino-cli core update-index --additional-urls https://per1234.github.io/ArduinoCore-sam/package_per1234_samarm64_index.json
export ARDUINO_BOARD_MANAGER_ADDITIONAL_URLS=https://per1234.github.io/ArduinoCore-sam/package_per1234_samarm64_index.json
arduino-cli core install per1234:sam

```

Microros Arduino Library

```
mkdir -p ~/Arduino/libraries
cd ~/Arduino/libraries
git clone -b humble https://github.com/micro-ROS/micro_ros_arduino.git

```
Now run the libgen.sh script to generate the custom library.
