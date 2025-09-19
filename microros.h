docker run -d --privileged --rm \
  --name microros_agent \
  --net=host \
  --env="ROS_DOMAIN_ID=42" \
  --device=/dev/ttyACM0 \
  microros/micro-ros-agent:humble \
  serial --dev /dev/ttyACM0 -b 115200