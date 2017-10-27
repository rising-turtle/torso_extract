
project(body_orientation)

###### interface transferring data from c++ to python ###
include_directories(/usr/include/python2.7)
add_library(toPython toPython.cpp)
target_link_libraries(toPython python2.7)
add_executable(test_toPython test_toPython.cpp)
target_link_libraries(test_toPython toPython ${OpenCV_LIBS})

###### relavent operations about camera model ######
add_library(cam_model cam_model.cpp rs_r200_wrapper.cpp)
target_link_libraries(cam_model ${OpenCV_LIBS})

###### read camera frames from disk and then extract torso #######
add_executable(torso_offline torso_offline.cpp global_def.cpp histogram_filter.cpp body_extract.cpp pt2img.cpp)
target_link_libraries(torso_offline cam_model toPython glfw GLU GL ${OpenCV_LIBS})

