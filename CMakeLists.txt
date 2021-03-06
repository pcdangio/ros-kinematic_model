cmake_minimum_required(VERSION 3.0.2)
project(kinematic_model)

# Find catkin and catkin package dependencies.
find_package(catkin REQUIRED COMPONENTS
  roscpp
  kalman_filter
  transform)

# Build catkin package.
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kinematic_model
  CATKIN_DEPENDS roscpp kalman_filter transform)

# Set up include directories.
include_directories(
  include
  ${catkin_INCLUDE_DIRS})

# Add kinematic_model library.
add_library(${PROJECT_NAME}
  src/kinematic_model/kinematic_model.cpp
  src/kinematic_model/geometry/object/object.cpp
  src/kinematic_model/geometry/object/link.cpp
  src/kinematic_model/geometry/object/frame.cpp
  src/kinematic_model/geometry/object/joint.cpp
  src/kinematic_model/geometry/attachment/attachment.cpp
  src/kinematic_model/geometry/attachment/fixed.cpp
  src/kinematic_model/geometry/attachment/dynamic.cpp
  src/kinematic_model/geometry/graph/graph.cpp
  src/kinematic_model/geometry/graph/connection.cpp
  src/kinematic_model/geometry/graph/vertex.cpp
  src/kinematic_model/geometry/design.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# Add kinematic_model node executable.
add_executable(${PROJECT_NAME}_node src/kinematic_model/main.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  dl)

# Install kinematic_model library and headers.
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

# Install kinematic_model node.
install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})