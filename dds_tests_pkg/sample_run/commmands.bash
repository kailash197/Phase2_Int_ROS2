cd ~/ros2_ws/
colcon build --packages-select dds_tests_pkg
source ~/ros2_ws/install/setup.bash
ros2 launch dds_tests_pkg subscriber_dds.launch.py

#shell 2
source ~/ros2_ws/install/setup.bash
ros2 launch dds_tests_pkg publisher_dds.launch.py

#or both 
source ~/ros2_ws/install/setup.bash
ros2 launch dds_tests_pkg start_publisher_subscriber.launch.py


#change dds and test
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run dds_tests_pkg subscriber_dds_exe

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run dds_tests_pkg publisher_dds_exe

# Both systems lose messages.
# These tests vary a lot depending on the system you execute them in. In this particular system, none of them seem to give a significant advantage.
# In bigger systems, FastDDS based, on your tests, outperforms Cyclone.


ps faux | grep "rmw-implementation"
ros2 daemon stop
