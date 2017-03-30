cmake_minimum_required (VERSION 3.1.0 FATAL_ERROR)
project (LocoMouse)
find_package (OpenCV REQUIRED)

# VERSION
set (LocoMouse_VERSION_MAJOR 1)
set (LocoMouse_VERSION_MINOR 0)

# Compiler options:
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_EXTENSIONS OFF)

# Configure file to get version number, optional flags, etc.
configure_file (
	"${PROJECT_SOURCE_DIR}/LocoMouseConfig.h.in"
	"${PROJECT_BINARY_DIR}/LocoMouseConfig.h"
	)

add_subdirectory(Candidates)
add_subdirectory(MyMat)
add_subdirectory(match2nd)
add_subdirectory(LocoMouse_Core)

include_directories(${Candidates_INCLUDE_DIRS} ${MyMat_INCLUDE_DIRS} ${match2nd_INCLUDE_DIRS} ${LocoMouse_Core_INCLUDE_DIRS})


add_executable(LocoMouse main.cpp)
target_link_libraries(LocoMouse ${OpenCV_LIBS} MyMat match2nd Candidates LocoMouse_Core) 