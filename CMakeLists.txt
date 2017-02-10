cmake_minimum_required (VERSION 3.1.0 FATAL_ERROR)
project (LocoMouse)
find_package (OpenCV REQUIRED)

# VERSION
set (LocoMouse_VERSION_MAJOR 1)
set (LocoMouse_VERSION_MINOR 0)

# Configure file to get version number, optional flags, etc.
configure_file (
	"${PROJECT_SOURCE_DIR}/LocoMouseConfig.h.in"
	"${PROJECT_BINARY_DIR}/LocoMouseConfig.h"
	)

add_subdirectory(Candidates)
add_subdirectory(MyMat)
add_subdirectory(match2nd)

include_directories(${Candidates_INCLUDE_DIRS} ${MyMat_INCLUDE_DIRS} ${match2nd_INCLUDE_DIRS})

# Compiler options:
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

add_executable(LocoMouse LocoMouse.cpp)
target_link_libraries(LocoMouse ${OpenCV_LIBS} MyMat match2nd Candidates)

# Compiler options:
#set(TARGET LocoMouse PROPERTY CMAKE_CXX_STANDARD 11)
#set(TARGET LocoMouse PROPERTY CMAKE_CXX_STANDARD_REQUIRED ON)
#set(TARGET LocoMouse PROPERTU CMAKE_BUILD_TYPE Release)


