# SOOKSHMA REPO

The build.sh script is to be used only for primary building the image on the system. The build.sh script will build multiplatform image - for amd64 and arm64. On all the RPIs use pull.sh  to pull the respective arm64 image.

Use start.sh to start the container and enter.sh to enter the container in another terminal.

libgen.sh used only on the system to generate the custom message compatible micro_ros_arduino library. The library has been already built and the image pushed, so the library will be available on the RPIs when image is pulled.

## Requirements for building the multi-platform image on the system

```
docker buildx create --name multiarch --driver docker-container --use 
docker run --privileged --rm tonistiigi/binfmt --install all 
docker buildx inspect --bootstrap

```
## Requirements for libgen on the system

Arduino CLI

```
cd ~
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

echo 'export PATH=$PATH:$HOME/bin' >> ~/.bashrc
source ~/.bashrc

arduino-cli core update-index
arduino-cli cache clean
arduino-cli core install arduino:sam@1.6.12
```

Microros Arduino Library

```
mkdir -p ~/Arduino/libraries
cd ~/Arduino/libraries
git clone -b humble https://github.com/micro-ROS/micro_ros_arduino.git

```
Now run the libgen.sh script to generate the custom library.
