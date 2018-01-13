
project(SCALE)

#### add ceres header and libraries ####
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_executable(estimate_scale estimate_scale.cpp)

target_link_libraries(estimate_scale ${CERES_LIBRARIES})
