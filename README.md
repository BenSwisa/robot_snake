using teensey 4.0 as MC

code for masters in micro_ros

make sure to launch the program in the SNAKE/SOURCE/ros2 when you record

make sure to source : source install/local_setup.bash

Launching micro_ros** checking available ports : ls -l /dev/ttyA* running: ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

github* Starting the ssh-agent: eval "$(ssh-agent -s)" ssh-add ~/.ssh/SNAKE_WS

libraries needed : RLSencoder2, teensy4_i2c, micro_ros_arduino
