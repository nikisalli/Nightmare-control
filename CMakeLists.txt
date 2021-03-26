cmake_minimum_required(VERSION 3.5)

project(Nightmare-control LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

find_package(roscpp REQUIRED)
find_package(rviz REQUIRED)
find_package(rosconsole REQUIRED)
find_package(rostime REQUIRED)

include_directories(/usr/include/OGRE)

# find_library(ROS_LIBRARY ros HINTS /opt/ros/noetic/lib)

include_directories(/opt/ros/noetic/include)
link_directories(/opt/ros/noetic/lib)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}) 

set(SOURCE_DIR src/)

set(PROJECT_SOURCES
        ${SOURCE_DIR}main.cpp
        ${SOURCE_DIR}mainwindow.cpp
        ${SOURCE_DIR}mainwindow.h
        ${SOURCE_DIR}myviz.cpp
        ${SOURCE_DIR}myviz.h
        ${SOURCE_DIR}mainwindow.ui
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(Nightmare-control
        ${PROJECT_SOURCES}
    )
else()
    add_executable(Nightmare-control
        ${PROJECT_SOURCES}
    )
endif()

target_link_libraries(Nightmare-control PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)
target_link_libraries(Nightmare-control PRIVATE roscpp)
target_link_libraries(Nightmare-control PRIVATE rviz)
target_link_libraries(Nightmare-control PRIVATE rosconsole)
target_link_libraries(Nightmare-control PRIVATE rostime)
# target_link_libraries(Nightmare-control PRIVATE ${ROS_LIBRARY})
# target_link_libraries(Nightmare-control PRIVATE OGRE)