project (match2nd)

include_directories(${MyMat_INCLUDE_DIRS})

add_library(match2nd match2nd.cpp)

target_link_libraries(match2nd MyMat)

# Make sure match2nd can be added:
set(match2nd_INCLUDE_DIRS ${PROJECT_SOURCE_DIR} ${MyMat_INCLUDE_DIRS}
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)