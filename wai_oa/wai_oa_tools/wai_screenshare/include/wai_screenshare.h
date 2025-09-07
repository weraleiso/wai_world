/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2019, W. A. Isop
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/




#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<sensor_msgs/image_encodings.h>
#include<cv_bridge/cv_bridge.h>

#include<opencv2/opencv.hpp>
#include<X11/Xlib.h>
#include<X11/Xutil.h>



class ScreenShot
{
    Display* display;
    Window root;
    int x,y,width,height;
    XImage* img{nullptr};

public:
    ScreenShot(int x, int y, int width, int height):
        x(x),
        y(y),
        width(width),
        height(height)
    {
        display = XOpenDisplay(nullptr);
        root = DefaultRootWindow(display);
    }

    void operator() (cv::Mat& cvImg)
    {
        if(img != nullptr)
            XDestroyImage(img);
        img = XGetImage(display, root, x, y, width, height, AllPlanes, ZPixmap);
        cvImg = cv::Mat(height, width, CV_8UC4, img->data);
    }

    ~ScreenShot()
    {
        if(img != nullptr)
            XDestroyImage(img);
        XCloseDisplay(display);
    }
};



class WAIScreenshare
{
    ros::NodeHandle hdl_node;
    image_transport::ImageTransport m_hdl_it;

    image_transport::Publisher m_pub_img_screenshot;
    sensor_msgs::ImagePtr m_msg_img_screenshot;
    cv::Mat m_mat_img_screenshot;

    std::string s_path_nodename;

    float F_NODE_SAMPLE_FREQUENCY;
    int I_SCREEN_OFFSET_X;
    int I_SCREEN_OFFSET_Y;
    int I_SCREEN_RESOLUTION_X;
    int I_SCREEN_RESOLUTION_Y;
    float F_SCREEN_RESOLUTION_SCALE;

public:
    WAIScreenshare();
    ~WAIScreenshare();

    void run();
};
