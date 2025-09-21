#docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -v6

# on the Pi host
docker run --rm -it \
  --name microros_agent \
  --net=host \
  --privileged \
  -e ROS_DOMAIN_ID=42 \
  --device=/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Due_Prog._Port_XXXXXXXX:/dev/ttyACM0 \
  microros/micro-ros-agent:humble \
  serial --dev /dev/ttyACM0 -b 115200 -v6
