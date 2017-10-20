
project(body_orientation)

###### relavent operations about camera model ######
add_library(cam_model cam_model.cpp rs_r200_wrapper.cpp)
target_link_libraries(cam_model ${OpenCV_LIBS})

###### read camera frames from disk and then extract torso #######
add_executable(torso_offline torso_offline.cpp global_def.cpp histogram_filter.cpp body_extract.cpp pt2img.cpp)
target_link_libraries(torso_offline cam_model glfw GLU GL ${OpenCV_LIBS})

