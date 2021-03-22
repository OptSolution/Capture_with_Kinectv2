/*
 * @Description:
 * @Author: Chen Wang
 * @Email: mr_cwang@foxmail.com
 * @since: 2021-03-22 10:36:32
 * @LastAuthor: Chen Wang
 * @lastTime: 2021-03-22 16:26:45
 */

#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>

#define CVUI_IMPLEMENTATION
#include "cvui.h"

int main(int argc, char const *argv[]) {
  // init kinect v2
  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;
  libfreenect2::PacketPipeline *pipeline = 0;

  // Discover the kinect v2
  if (freenect2.enumerateDevices() == 0) {
    std::cerr << "no device connected!" << std::endl;
    return -1;
  }

  std::string serial = freenect2.getDefaultDeviceSerialNumber();
  std::cout << "Serial is " << serial << std::endl;

  if (!pipeline) {
    pipeline = new libfreenect2::OpenGLPacketPipeline();
  } else {
    std::cerr << "pipeline is error" << std::endl;
    return -1;
  }

  // open the device
  if (pipeline) {
    dev = freenect2.openDevice(serial, pipeline);
  } else {
    dev = freenect2.openDevice(serial);
  }

  if (dev == 0) {
    std::cerr << "failure opening device!" << std::endl;
    return -1;
  }

  // Configure the device
  bool enable_rgb = true;
  bool enable_depth = true;

  if (!enable_rgb && !enable_depth) {
    std::cerr << "Disabling both streams is not allowed!" << std::endl;
    return -1;
  }

  int types = 0;
  if (enable_rgb)
    types |= libfreenect2::Frame::Color;
  if (enable_depth)
    types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;

  libfreenect2::SyncMultiFrameListener listener(types);
  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);

  // Start the device
  if (enable_rgb && enable_depth) {
    if (!dev->start())
      return -1;
  } else {
    if (!dev->startStreams(enable_rgb, enable_depth))
      return -1;
  }
  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

  // TODO : set the params
  libfreenect2::Registration *registration = new libfreenect2::Registration(
      dev->getIrCameraParams(), dev->getColorCameraParams());
  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4) /*,
       depth2rgb(1920, 1080 + 2, 4)*/
      ;

  // new window
  cvui::init("Capture with Kinect v2");

  // set opencv image
  cv::Mat rgb_cv, depth_cv, ir_cv, undistroted_depth_cv /*,depth2rgb_cv,
      rgb2depth_cv*/
      ;

  // listening
  bool protonect_shutdown = false;
  bool enable_filter = true;
  libfreenect2::FrameMap frames;
  while (!protonect_shutdown) {
    // 10 seconds
    if (!listener.waitForNewFrame(frames, 10 * 1000)) {
      std::cerr << "timeout!" << std::endl;
      return -1;
    }

    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth =
        frames[libfreenect2::Frame::Depth]; // unit: millimeter

    // TODO : align depth and rgb
    if (enable_rgb && enable_depth) {
      registration->apply(rgb, depth, &undistorted, &registered, true/*,
                          &depth2rgb*/);
    }

    // convert images to opencv mat
    cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgb_cv);
    cv::Mat(depth->height, depth->width, CV_32FC1, depth->data)
        .copyTo(depth_cv);
    cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(ir_cv);
    cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data)
        .copyTo(undistroted_depth_cv);

    // release the frames
    listener.release(frames);

    // esc to quit
    if (cv::waitKey(20) == 27) {
      protonect_shutdown = true;
    }
  }

  dev->stop();
  dev->close();

  return 0;
}
