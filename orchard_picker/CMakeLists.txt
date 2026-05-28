cmake_minimum_required(VERSION 3.5)
project(orchard_picker)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetArmState.srv"
  "srv/GripperPick.srv"
  "srv/GripperPosition.srv"
  "srv/GripperRelease.srv"
  "srv/MoveJ.srv"
  "srv/MovePose.srv"
  "srv/RawJson.srv"
  "srv/SetToolVoltage.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/fake_apple_target.py
  scripts/pick_task_manager.py
  scripts/raw_json_cli.py
  scripts/rm_json_bridge.py
  scripts/t930_arm_net_check.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
