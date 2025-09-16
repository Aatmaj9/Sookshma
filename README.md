# SOOKSHMA REPO

build.sh script is to be used only for primary building the image on the system. The build.sh script will build multiplatform image - for amd64 and arm64. On all the RPIs use pull.sh  to pull the respective arm64 image.
libgen.sh used only on the system to generate the custom message compatible micro_ros_arduino library. The library has alredy built and the image pushed and so will be available on the RPIs when image is pulled.

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
