# robot_snake

to use this library you need to install [ros2 humble](https://docs.ros.org/en/humble/Installation.html), and [micro ros](https://micro.ros.org/docs/tutorials/core/first_application_linux/)

make sure to source for ros2 and for microros : source install/local_setup.bash
either source or add in ~/.bashrc : source ~/microros_ws/install/local_setup.bash 

Launching micro_ros** checking available ports : ls -l /dev/ttyA* running: ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

github* Starting the ssh-agent: eval "$(ssh-agent -s)" ssh-add ~/.ssh/robot_snake

libraries needed : RLSencoder2, teensy4_i2c, micro_ros_arduino

source /opt/ros/humble/setup.bash

make sure to allow dialout : sudo usermod -a -G dialout $USER


ARDUINO IDE: (to change the arduino nodes you need to set up your arudino ide) 
- install [teensyduino]( https://www.pjrc.com/teensy/td_download.html)
- install [teensy4_i2c](https://github.com/Richard-Gemmell/teensy4_i2c)
- add the rls_encoder2 library from the package1/lib folder 
- install [micro_ros_arduino](https://github.com/micro-ROS/micro_ros_arduino/releases)
- install [hx711_multi](https://github.com/compugician/HX711-multi.git)

TOPIC CRASHING(This happens when some node unexpectedly crashed previously)-> 
restart daemon:
```
ros2 daeomon stop 

ros2 daemon start
```

MICRO-ROS:
 - finding teensey id : 
 ```
 ls -l /dev/serial/by-id/
 ```
 
 
 to record : 
 ```
  ros2 launch package1 launch1.py 'record:=True'
 ```
