// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

///////////////////////////////////////////////////////////
// librealsense tutorial #2 - Accessing multiple streams //
///////////////////////////////////////////////////////////

/*
 *  Oct. 13 2017 David Z 
 *
 *  Save rgbd data
 *
 * */



// First include the librealsense C++ header file
#include <librealsense/rs.hpp>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
// Also include GLFW to allow for graphical display
#include <GLFW/glfw3.h>
#include <cv.h>
#include <highgui.h>
#include <string>
#include <sstream>

using namespace cv; 
using namespace std;

inline string fname(int id)
{
  stringstream ss; 
  ss << setfill('0')<<setw(6)<<id<<".png"; 
  return ss.str();
} 

int main(int argc, char* argv[]) try
{
    // Create a context object. This object owns the handles to all connected realsense devices.
    rs::context ctx;
    printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
    if(ctx.get_device_count() == 0) return EXIT_FAILURE;

    // This tutorial will access only a single device, but it is trivial to extend to multiple devices
    rs::device * dev = ctx.get_device(0);
    printf("\nUsing device 0, an %s\n", dev->get_name());
    printf("    Serial number: %s\n", dev->get_serial());
    printf("    Firmware version: %s\n", dev->get_firmware_version());

    int fps = 30; 

    // Configure all streams to run at VGA resolution at 60 frames per second
    dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, fps);
    dev->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, fps);
    // dev->enable_stream(rs::stream::infrared, 640, 480, rs::format::y8, 60);
    // try { dev->enable_stream(rs::stream::infrared2, 640, 480, rs::format::y8, 60); }
    // catch(...) { printf("Device does not provide infrared2 stream.\n"); }
    dev->start();

    // Open a GLFW window to display our output
    glfwInit();
    GLFWwindow * win = glfwCreateWindow(1280, 480, "librealsense tutorial #2", nullptr, nullptr);
    glfwMakeContextCurrent(win);

    // where to save image 
    string dir("./data"); 
    if(argc > 1) 
      dir = argv[1]; 
    int id = 1; 
    string c_dir = dir + "/color";
    string d_dir = dir + "/depth";
    mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH); 
    mkdir(c_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH); 
    mkdir(d_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH); 

    while(!glfwWindowShouldClose(win))
    {
        // Wait for new frame data
        glfwPollEvents();
        dev->wait_for_frames();

        glClear(GL_COLOR_BUFFER_BIT);
        glPixelZoom(1, -1);

        // Display depth data by linearly mapping depth between 0 and 2 meters to the red channel
        glRasterPos2f(-1, 1);
        glPixelTransferf(GL_RED_SCALE, 0xFFFF * dev->get_depth_scale() / 2.0f);
        glDrawPixels(640, 480, GL_RED, GL_UNSIGNED_SHORT, dev->get_frame_data(rs::stream::depth));
        glPixelTransferf(GL_RED_SCALE, 1.0f);

        // Display color image as RGB triples
        glRasterPos2f(0, 1);
        glDrawPixels(640, 480, GL_RGB, GL_UNSIGNED_BYTE, dev->get_frame_data(rs::stream::color));

        // Display infrared image by mapping IR intensity to visible luminance
        // glRasterPos2f(-1, 0);
        // glDrawPixels(640, 480, GL_LUMINANCE, GL_UNSIGNED_BYTE, dev->get_frame_data(rs::stream::infrared));

        // Display second infrared image by mapping IR intensity to visible luminance
        // if(dev->is_stream_enabled(rs::stream::infrared2))
        {        
           // glRasterPos2f(0, 0);
           // glDrawPixels(640, 480, GL_LUMINANCE, GL_UNSIGNED_BYTE, dev->get_frame_data(rs::stream::infrared2));
        }

        // generate rgb cv::Mat 
        cv::Mat rgb(480, 640, CV_8UC3, (void*)(dev->get_frame_data(rs::stream::color))); 
        // cv::imshow("rgb", rgb); 
        // cv::waitKey(20); 

        // generate depth cv::Mat
        cv::Mat dpt(480, 640, CV_16UC1, (void*)(dev->get_frame_data(rs::stream::depth)));

        // save data 
        string c_name = c_dir + "/" + fname(id); 
        string d_name = d_dir + "/" + fname(id); 
        cv::imwrite(c_name.c_str(), rgb);
        cv::imwrite(d_name.c_str(), dpt);
        id++; 
        cout <<"save_img.cpp: save "<<id<<" images!"<<endl; 

        // cv::imshow("dpt", dpt);
        // cv::waitKey(20); 
    
        glfwSwapBuffers(win);
        usleep(1000*10); 
    }
    
    return EXIT_SUCCESS;
}
catch(const rs::error & e)
{
    // Method calls against librealsense objects may throw exceptions of type rs::error
    printf("rs::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
    printf("    %s\n", e.what());
    return EXIT_FAILURE;
}
