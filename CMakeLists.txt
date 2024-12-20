cmake_minimum_required(VERSION 3.8)
project(ip_tunnel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Set dependencies
set(dependencies
    rclcpp
    std_msgs
)

# Add executables
add_executable(test_tun_transmit src/test_tun_transmit.cpp)
add_executable(test_tun_receive src/test_tun_receive.cpp)
add_executable(test_tun_receive_headers src/test_tun_receive_headers.cpp)
add_executable(test_multiarray src/test_multiarray.cpp)
add_executable(ip_tunnel_node src/ip_tunnel_node.cpp)

# Add dependencies
ament_target_dependencies(test_multiarray ${dependencies})
ament_target_dependencies(ip_tunnel_node ${dependencies})

# Install C++ executables
install(TARGETS
  test_tun_transmit
  test_tun_receive
  test_tun_receive_headers
  test_multiarray
  ip_tunnel_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts with exec permission
install(PROGRAMS
  script/test_udp_transmit
  script/test_udp_receive
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
