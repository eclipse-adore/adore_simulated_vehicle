cmake_minimum_required(VERSION 3.21)

get_filename_component(CURRENT_DIR_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(TOLOWER ${CURRENT_DIR_NAME} PROJECT)
project(${PROJECT})

include($ENV{SOURCE_DIRECTORY}/libraries/adore_libraries.cmake)

include(requirements.cmake)
set(CMAKE_CXX_STANDARD 20)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
# Ensure the Release build uses -O3 for optimization
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_executable(${PROJECT}  src/simulated_vehicle_node.cpp)


target_include_directories(${PROJECT} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


add_all_target_include_directories(${PROJECT})
add_all_target_link_libraries(${PROJECT})



install(TARGETS
    ${PROJECT}
  DESTINATION lib/${PROJECT}
)

add_library(simulatedvehiclenode_component SHARED src/simulated_vehicle_node.cpp)
add_all_target_include_directories(simulatedvehiclenode_component)
add_all_target_link_libraries(simulatedvehiclenode_component)
rclcpp_components_register_node(
  simulatedvehiclenode_component
  PLUGIN "adore::simulated_vehicle::SimulatedVehicleNode"
  EXECUTABLE simulatedvehiclenode
)
install(TARGETS simulatedvehiclenode_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_package()
