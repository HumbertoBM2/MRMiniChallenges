cmake_minimum_required(VERSION 3.5)
project(signal_processing_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(signal_generator src/signal_generator.cpp)
ament_target_dependencies(signal_generator rclcpp std_msgs)

add_executable(process src/process.cpp)
ament_target_dependencies(process rclcpp std_msgs)

# Install executables
install(TARGETS
  signal_generator
  process
  DESTINATION lib/${PROJECT_NAME})

# Install launch file
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
