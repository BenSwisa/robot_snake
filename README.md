# robot_snake
using teensey 4.0 as MC

code for masters in micro_ros

make sure to launch the program in the SNAKE/SOURCE/ros2 when you record

make sure to source for ros2 and for microros : source install/local_setup.bash
either source or add in ~/.bashrc : source ~/microros_ws/install/local_setup.bash 

Launching micro_ros** checking available ports : ls -l /dev/ttyA* running: ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

github* Starting the ssh-agent: eval "$(ssh-agent -s)" ssh-add ~/.ssh/robot_snake

libraries needed : RLSencoder2, teensy4_i2c, micro_ros_arduino

source /opt/ros/humble/setup.bash

make sure to allow dialout : sudo usermod -a -G dialout $USER

ARDUINO IDE:
- install teensyduino : https://www.pjrc.com/teensy/td_download.html
- install teensy4_i2c : https://github.com/Richard-Gemmell/teensy4_i2c
- add and include rls_encoder2 library : 

