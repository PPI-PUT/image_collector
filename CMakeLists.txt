cmake_minimum_required(VERSION 3.14)
project(image_collector)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)
add_compile_options(-Wextra -pedantic -Werror)

set(IMAGE_COLLECTOR_LIB_SRC
  src/image_collector.cpp
)

set(IMAGE_COLLECTOR_LIB_HEADERS
  include/image_collector/image_collector.hpp
  include/image_collector/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${IMAGE_COLLECTOR_LIB_SRC}
  ${IMAGE_COLLECTOR_LIB_HEADERS}
)

set(IMAGE_COLLECTOR_NODE_SRC
  src/image_collector_node.cpp
)

set(IMAGE_COLLECTOR_NODE_HEADERS
  include/image_collector/image_collector_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${IMAGE_COLLECTOR_NODE_SRC}
  ${IMAGE_COLLECTOR_NODE_HEADERS}
)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "image_collector::ImageCollectorNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)
