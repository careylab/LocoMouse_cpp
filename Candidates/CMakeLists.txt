cmake_minimum_required (VERSION 2.8)

project(Candidates)

find_package (OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(Candidates Candidates.cpp)

target_link_libraries(Candidates ${OpenCV_LIBS})

# To include Candidates in top level project:
set(Candidates_INCLUDE_DIRS ${PROJECT_SOURCE_DIR} CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)