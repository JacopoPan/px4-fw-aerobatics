#!/bin/bash

ros2 service call /do_the_thing rcl_interfaces/srv/GetParameters "{names: ['takeoff']}"

sleep 50
ros2 service call /do_the_thing rcl_interfaces/srv/GetParameters "{names: ['change_alt']}"

sleep 5
ros2 service call /do_the_thing rcl_interfaces/srv/GetParameters "{names: ['change_speed']}"

sleep 55
ros2 service call /do_the_thing rcl_interfaces/srv/GetParameters "{names: ['reposition']}"

sleep 20
ros2 service call /do_the_thing rcl_interfaces/srv/GetParameters "{names: ['roll']}"

sleep 20
ros2 service call /do_the_thing rcl_interfaces/srv/GetParameters "{names: ['dive']}"

sleep 20
ros2 service call /do_the_thing rcl_interfaces/srv/GetParameters "{names: ['land']}"