# robot_snake

to use this library you need to install [ros2 humble](https://docs.ros.org/en/humble/Installation.html), and [micro ros](https://micro.ros.org/docs/tutorials/core/first_application_linux/)

## ros2
to run the program:

first make sure nothing is connected.

now source ros2 workspace and microros workspace:
```
source /opt/ros/humble/setup.bash
source ~/microros_ws/install/local_setup.bash
source ~/robot_snake/install/local_setup.bash
```
run micro ros agent launch file:
```
  ros2 launch package1 m_r_launch.py
```
run launch1
```
ros2 launch package1 launch1.py

```
if you want to record add record argument:
```
  ros2 launch package1 launch1.py 'record:=True'
```

to plot a joint value in position 0 in real time use rqt->visualize->plot and add "joint_val_topic/data[0]"
 

## Arduino IDE: 
(to change the arduino nodes you need to set up your arudino ide) 

first follow the step in the [arduino website](https://docs.arduino.cc/software/ide-v1/tutorials/Linux) 
make sure to allow dialout : (if you havent done while setting your arduino ide)
```
 sudo usermod -a -G dialout $USER
```
now
- install [teensyduino]( https://www.pjrc.com/teensy/td_download.html)
- install [teensy4_i2c](https://github.com/Richard-Gemmell/teensy4_i2c)
- add the rls_encoder2 library from the package1/lib folder 
- install [micro_ros_arduino](https://github.com/micro-ROS/micro_ros_arduino/releases)
- install [hx711_multi](https://github.com/compugician/HX711-multi.git)

## known issues

TOPIC CRASHING (This happens when some node unexpectedly crashed previously)-> 
restart daemon:
```
ros2 daeomon stop 

ros2 daemon start
```
- if daemon crashes frequently try:
```
RMW_IMPLEMENTATION=rmw_connext_cpp
```
or look for an answer in one of the following topics: https://github.com/ros2/rmw_fastrtps/issues/509 , https://github.com/ros2/ros2cli/issues/582

- if joint_val is not publishing correctly try rebooting the robot using the big red button
- if rqt not responding acuuratly try re-oprnig rqt
## MICRO-ROS:
 - finding teensey id : 
 ```
 ls -l /dev/serial/by-id/
 ```
 
github* Starting the ssh-agent: eval "$(ssh-agent -s)" ssh-add ~/.ssh/robot_snake
 
