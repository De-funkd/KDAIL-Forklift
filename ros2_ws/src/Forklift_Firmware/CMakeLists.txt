cmake_minimum_required(VERSION 3.8)
project(Forklift_Firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  # src/imu_publisher.py
  # src/uwb_to_odom.py
  # src/dummy_uwb_publisher.py
  # src/camera_init.py
  # src/imu_pub.py
  # src/aruco-detection-and-streaming.py
  src/imu_esp_pub.py
  src/ism_imu_pub.py
  # src/uwb_xy.py
  src/motor_controller.py
  src/raw_uwb_publish.py
  src/tests/single_uwb_publish.py
  src/tests/ui_display.py
  # src/tests/uwb_xy.py
  src/tests/uwb_xy_ui.py
  src/tests/uwb_xy.py
  src/uwb/uwb_data_parser.py
  src/uwb/uwb_trilateration_test.py
  src/uwb/xy_ui_uwb.py
  src/uwb/uwb_trilateration.py
  # src/uwb_to_odom.py
  # src/uwb_fusion/uwb_imu_fusion.py
  # src/uwb_fusion/uwb_raw_publisher.py
  # src/uwb_fusion/uwb_fusion_test.py
  # src/dynamic_uwb_positioning.py
  # src/test_uwb_imu_publisher.py
  # src/coordinate_system_visualizer.py
  # src/uwb_imu_transform_publisher.py
  # src/uwb_static_slam.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  rviz
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
