cmake_minimum_required(VERSION 3.8)
project(face_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 実行可能ファイルの追加
add_executable(face_detector src/face_detector.cpp)

# 必要なライブラリのリンク
ament_cmake_export_dependencies(face_detector
  rclcpp
  sensor_msgs
  cv_bridage
  OpenCV
)

# インストール設定
install(TARGETS
  face_detector
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
