cmake_minimum_required(VERSION 3.8)
project(robstride_dk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robstride_motor_msgs REQUIRED)  # 🔹 추가: msg 패키지

set(TARGET_NAME robstride_dk_node)

add_executable(${TARGET_NAME}
  src/robstride_dk_node.cpp
  src/robstride_driver.cpp
)

target_include_directories(${TARGET_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# C++만 쓰니까 cxx_std_17만 있어도 충분
target_compile_features(${TARGET_NAME} PUBLIC cxx_std_17)

# 🔹 여기서부터만 살짝 변경
#   - target_link_libraries 대신 ament_target_dependencies 사용
#   - rclcpp, std_msgs, robstride_motor_msgs 전부 의존성으로 등록
ament_target_dependencies(${TARGET_NAME}
  rclcpp
  std_msgs
  robstride_motor_msgs
)

install(TARGETS ${TARGET_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
