#!/bin/bash

# DDS Topics
cp ~/git/ws/src/px4-fw-aerobatics/resources/dds_topics.yaml ~/git/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml 

# ROMS
cp -r ~/git/ws/src/px4-fw-aerobatics/resources/airframes ~/git/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/
cd ~/git/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
rm CMakeLists.txt
echo "px4_add_romfs_files(" >> CMakeLists.txt && ls | sed 's/^/  /' >> CMakeLists.txt && echo ")" >> CMakeLists.txt

# Rebuild
cd ~/git/PX4-Autopilot
rm -rf build/
make px4_sitl 