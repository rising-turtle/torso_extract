# torso_extract
Extract torso and compute body orientation

The details and result can be found in the paper:

* M. R. Afsar1, M. Wadsworth, T. Shen, H. Zhang, C. Ye, X. Shen, **“A Motorized Robotic Walker for Human Walking Assistance,”**
ASME Journal of Medical Devices, 2017.

## src
Main functions, include read data from camera, histogram filter, smooth (average filter), polyfit (curve), point cloud display
**body_orientation_serial.cpp**   
**body_orientation_serial_curve.cpp**

## ipynb 
Contains a polyfit function written in python script 

## librealsense
Library of functions to access to realsense camera, including RealSense R200

## offline 
Test the main functions from the data in disk.

## read_serial_src & win_recv_serial 
Test of sending and receiving data through serial port 

## scale 
Estimate a scale value to align the estiamted angle with the true angle obtained from an external IMU
