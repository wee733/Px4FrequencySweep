cmake_minimum_required(VERSION 3.8)
project(px4_frequency_sweep)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_ros2_cpp REQUIRED)

add_library(frequency_sweep_algorithm
  src/sweep_generator.cpp
)
target_include_directories(frequency_sweep_algorithm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(frequency_sweep_mode
  src/configuration.cpp
  src/csv_logger.cpp
  src/frequency_sweep_mode.cpp
  src/main.cpp
)
target_include_directories(frequency_sweep_mode PRIVATE
  include
  ${Eigen3_INCLUDE_DIRS}
)
target_link_libraries(frequency_sweep_mode PRIVATE
  frequency_sweep_algorithm
  Eigen3::Eigen
  px4_ros2_cpp::px4_ros2_cpp
  rclcpp::rclcpp
)

install(TARGETS
  frequency_sweep_algorithm
  frequency_sweep_mode
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(sweep_generator_test test/sweep_generator_test.cpp)
  target_include_directories(sweep_generator_test PRIVATE include)
  target_link_libraries(sweep_generator_test PRIVATE frequency_sweep_algorithm)
  add_test(NAME sweep_generator_test COMMAND sweep_generator_test)
endif()

ament_package()
