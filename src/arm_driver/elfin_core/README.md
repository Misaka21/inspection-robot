# elfin_core (minimal native core)

This folder only keeps the low-level EtherCAT C/C++ core used by the Elfin arm.

Design boundary:

- `elfin_core/`: vendor C/C++ core only
- `arm_driver/src/`: project ROS wrappers and interface mapping

Included:

- `soem_ros2` (SOEM EtherCAT core sources)
- `elfin_ethercat_driver` (native Elfin EtherCAT manager/client classes)

Excluded on purpose:

- MoveIt/basic_api related packages
- bringup/launch/config packages
- description/gazebo/model packages
- ros_control hardware plugin packages

Notes:

- IO service extension (`elfin_ethercat_io_client`) is removed to keep the core focused on arm joints.
- ROS node executable entry from vendor (`elfin_ethercat_driver_node.cpp`) is removed.
- Any ROS API exposure should be implemented in `arm_driver/src`.
