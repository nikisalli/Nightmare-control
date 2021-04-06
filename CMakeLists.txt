cmake_minimum_required(VERSION 3.5)

project(Nightmare-control LANGUAGES CXX)

find_package(roscpp REQUIRED)
find_package(rviz REQUIRED)
find_package(rosconsole REQUIRED)
find_package(rostime REQUIRED)
find_package(Curses REQUIRED)

add_library( cdk SHARED IMPORTED )
set_target_properties( cdk PROPERTIES IMPORTED_LOCATION /usr/lib/libcdk.so )

include_directories(/opt/ros/noetic/include)
link_directories(/opt/ros/noetic/lib)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}) 

set(SOURCE_DIR src/)

set(PROJECT_SOURCES
    ${SOURCE_DIR}main.cpp
)

add_executable(Nightmare-control ${PROJECT_SOURCES})

target_link_libraries(Nightmare-control PRIVATE roscpp)
target_link_libraries(Nightmare-control PRIVATE rviz)
target_link_libraries(Nightmare-control PRIVATE rosconsole)
target_link_libraries(Nightmare-control PRIVATE rostime)
target_link_libraries(Nightmare-control PRIVATE roscpp_serialization)
target_link_libraries(Nightmare-control PRIVATE pthread)
target_link_libraries(Nightmare-control PRIVATE curses)
target_link_libraries(Nightmare-control PRIVATE cdk)