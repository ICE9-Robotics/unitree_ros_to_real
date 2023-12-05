cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_real)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    sensor_msgs
    unitree_legged_msgs
    nav_msgs
    message_runtime
)

add_service_files(
  FILES
    SetUnitreeHLMode.srv
)

generate_messages()

catkin_package()

message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()

link_directories(${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/cpp/${ARCH})

set(EXTRA_LIBS -pthread libunitree_legged_sdk.so)

set(CMAKE_CXX_FLAGS "-O3 -fPIC")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/include
)

add_executable(unitree_high_level src/exe/unitree_high_level.cpp)
target_link_libraries(unitree_high_level ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(unitree_high_level ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(unitree_odom_tf_fix_node src/exe/unitree_odom_tf_fix_node.cpp)
target_link_libraries(unitree_odom_tf_fix_node ${catkin_LIBRARIES})
