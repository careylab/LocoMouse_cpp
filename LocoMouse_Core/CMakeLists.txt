cmake_minimum_required (VERSION 2.8)

project(LocoMouse_Core)

find_package (OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${Candidates_INCLUDE_DIRS} ${MyMat_INCLUDE_DIRS} ${match2nd_INCLUDE_DIRS})

file(GLOB SOURCES "*.cpp")

add_library(LocoMouse_Core ${SOURCES})

target_link_libraries(LocoMouse_Core ${OpenCV_LIBS} Candidates MyMat match2nd)

# To include LocoMouse_Core in top level project:
set(LocoMouse_Core_INCLUDE_DIRS ${PROJECT_SOURCE_DIR} CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)