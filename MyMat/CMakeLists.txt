project(MyMat)
add_library(MyMat MyMat.cpp)

set_target_properties(MyMat PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_link_libraries(MyMat)

target_include_directories(MyMat PUBLIC ${PROJECT_SOURCE_DIR})

# To include MyMat in top level project:
set(MyMat_INCLUDE_DIRS ${PROJECT_SOURCE_DIR} CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)