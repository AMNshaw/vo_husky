cmake_minimum_required(VERSION 3.10)
project(vo_husky)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(gazebo_msgs REQUIRED)

# Eigen
find_package(Eigen3 REQUIRED NO_MODULE)
# include_directories("/usr/include/eigen3")
include_directories(${EIGEN3_INCLUDE_DIR})

# OpenCV
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# Ceres
find_package(Ceres REQUIRED)

# G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# gtest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# gflags
find_package(GFlags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
find_package(VTK REQUIRED)
include_directories(/usr/include/vtk-9.1)
include_directories(${VTK_INCLUDE_DIRS})
link_directories(${VTK_LIBRARY_DIRS})
add_definitions(${VTK_DEFINITIONS})

SET(G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core cxsparse)
include_directories(${PROJECT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})

include_directories(include)

# camera 
add_library(camera SHARED
  src/camera.cpp
)
ament_export_targets(camera HAS_LIBRARY_TARGET) 
install(
  TARGETS camera 
  EXPORT camera 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# frame
add_library(frame SHARED
  src/frame.cpp
)
ament_export_targets(frame HAS_LIBRARY_TARGET) 
install(
  TARGETS frame 
  EXPORT frame 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# feature
add_library(feature SHARED
  src/feature.cpp
)
ament_export_targets(feature HAS_LIBRARY_TARGET) 
install(
  TARGETS feature 
  EXPORT feature 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# mapPoint
add_library(mapPoint SHARED
  src/mapPoint.cpp
)
ament_export_targets(mapPoint HAS_LIBRARY_TARGET) 
install(
  TARGETS mapPoint 
  EXPORT mapPoint 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# map
add_library(map SHARED
  src/map.cpp
)
ament_export_targets(map HAS_LIBRARY_TARGET) 
install(
  TARGETS map 
  EXPORT map 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# frontend
add_library(frontend SHARED
  src/frontend.cpp
)
ament_export_targets(frontend HAS_LIBRARY_TARGET) 
install(
  TARGETS frontend 
  EXPORT frontend 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
target_link_libraries(frontend
  camera
  frame
  feature
  mapPoint
  map
  viewer
  backend
  ${CERES_LIBRARIES}
)

# backend
add_library(backend SHARED
  src/backend.cpp
)
ament_export_targets(backend HAS_LIBRARY_TARGET) 
install(
  TARGETS backend 
  EXPORT backend 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
target_link_libraries(backend
  camera
  frame
  feature
  mapPoint
  map
  ${G2O_LIBRARIES} 
  g2o_core g2o_stuff g2o_types_sba  
  g2o_types_slam3d 
  g2o_solver_csparse g2o_csparse_extension
  cholmod g2o_solver_cholmod
)

# visual odometry
add_library(visual_odometry SHARED
  src/visual_odometry.cpp
)
ament_export_targets(visual_odometry HAS_LIBRARY_TARGET) 
install(
  TARGETS visual_odometry 
  EXPORT visual_odometry 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
target_link_libraries(visual_odometry
  camera
  frame
  feature
  mapPoint
  map
  frontend
  backend
  viewer
  ${OpenCV_LIBS}
)

# viewer
add_library(viewer SHARED
  src/viewer.cpp
)
ament_export_targets(viewer HAS_LIBRARY_TARGET) 
install(
  TARGETS viewer 
  EXPORT viewer 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
target_link_libraries(viewer
  map
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES} GL GLU GLEW glut
  ${GTEST_BOTH_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
)

# camera_data_sub
add_library(camera_data_sub SHARED
  src/ros2_related/camera_data_sub.cpp
)
target_link_libraries(camera_data_sub
  camera
  frame
  feature
  ${OpenCV_LIBS}
)
ament_target_dependencies(camera_data_sub 
  rclcpp 
  std_msgs 
  sensor_msgs
  cv_bridge
  message_filters
)
ament_export_targets(camera_data_sub HAS_LIBRARY_TARGET) 
install(
  TARGETS camera_data_sub 
  EXPORT camera_data_sub 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# gazebo_gt_sub
add_library(gazebo_gt_sub SHARED
  src/ros2_related/gazebo_gt_sub.cpp
)
target_link_libraries(gazebo_gt_sub
  camera
  frame
  feature
  ${OpenCV_LIBS}
)
ament_target_dependencies(gazebo_gt_sub 
  rclcpp 
  std_msgs 
  gazebo_msgs
)
ament_export_targets(gazebo_gt_sub HAS_LIBRARY_TARGET) 
install(
  TARGETS gazebo_gt_sub 
  EXPORT gazebo_gt_sub 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# main_node
add_executable(main_node node/main_node.cpp)
target_link_libraries(main_node
  camera_data_sub
  gazebo_gt_sub
  visual_odometry
)

install(TARGETS 
  main_node
  DESTINATION lib/${PROJECT_NAME})


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
