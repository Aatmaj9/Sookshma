sudo cp -r ros2_ws/src/interfaces ~/Arduino/libraries/micro_ros_arduino/extras/library_generation/extra_packages/

cd ~/Arduino/libraries/micro_ros_arduino

docker run -it --rm -v $(pwd):/project \
--env MICROROS_LIBRARY_FOLDER=extras \
microros/micro_ros_static_library_builder:humble \
-p cortex_m3 -t serial 

sudo chown -R $(whoami) ~/Arduino/libraries/micro_ros_arduino

