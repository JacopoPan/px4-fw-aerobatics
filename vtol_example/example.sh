#!/bin/bash

ros2 service call /do_the_thing rcl_interfaces/srv/GetParameters "{names: ['takeoff']}"

sleep 10
ros2 service call /do_the_thing rcl_interfaces/srv/GetParameters "{names: ['reposition']}"

sleep 10
ros2 service call /do_the_thing rcl_interfaces/srv/GetParameters "{names: ['change_alt']}"

sleep 5
ros2 service call /do_the_thing rcl_interfaces/srv/GetParameters "{names: ['change_speed']}"

### offboard phases

sleep 10
ros2 service call /do_the_thing rcl_interfaces/srv/GetParameters "{names: ['traj']}"
sleep 10
ros2 service call /do_the_thing rcl_interfaces/srv/GetParameters "{names: ['reposition']}"

sleep 10
ros2 service call /do_the_thing rcl_interfaces/srv/GetParameters "{names: ['roll']}"
sleep 10
ros2 service call /do_the_thing rcl_interfaces/srv/GetParameters "{names: ['reposition']}"

sleep 10
ros2 service call /do_the_thing rcl_interfaces/srv/GetParameters "{names: ['dive']}"
sleep 10
ros2 service call /do_the_thing rcl_interfaces/srv/GetParameters "{names: ['reposition']}"

###

sleep 20
ros2 service call /do_the_thing rcl_interfaces/srv/GetParameters "{names: ['land']}"