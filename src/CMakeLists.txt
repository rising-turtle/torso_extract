
project(body_orientation)


######## show the point cloud generated using realsense camera ######
# add_executable(show_pc show_pc.cpp)
# target_link_libraries(show_pc glfw realsense GLU GL)

######## show the body direction computed using plane fitting ######
add_executable(body_orientation body_orientation.cpp body_extract.cpp global_def.cpp histogram_filter.cpp crc_generator.cpp coder.cpp)
 target_link_libraries(body_orientation glfw realsense GLU GL ${OpenCV_LIBS})

######## show the body direction computed using plane fitting ######
 add_executable(body_orientation2 body_orientation_m2.cpp body_extract.cpp global_def.cpp)
 target_link_libraries(body_orientation2 glfw realsense GLU GL)

 ######## test the histogram ######
 add_executable(test_histogram test_histogram.cpp body_extract.cpp global_def.cpp histogram_filter.cpp)
 target_link_libraries(test_histogram glfw realsense GLU GL ${OpenCV_LIBS})

######## a test for serial communication ########
 add_executable(test_serial test_serial.cpp serial_com.cpp)

######## compute body direction and send it through serial port (default: /dev/ttyUSB0) ###### 
add_executable(body_orientation_serial body_orientation_serial.cpp body_extract.cpp global_def.cpp serial_com.cpp histogram_filter.cpp crc_generator.cpp coder.cpp)
target_link_libraries(body_orientation_serial realsense ${OpenCV_LIBS})

###### view and save rgb & dpt data from rs200 #####
add_executable(save_img save_img.cpp global_def.cpp)
target_link_libraries(save_img glfw realsense GLU GL ${OpenCV_LIBS})
