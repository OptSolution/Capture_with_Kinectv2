/*
 * @Description:
 * @Author: Chen Wang
 * @Email: mr_cwang@foxmail.com
 * @since: 2021-03-22 10:36:32
 * @LastAuthor: Chen Wang
 * @lastTime: 2021-03-25 17:03:19
 */

#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>

#include <opencv2/opencv.hpp>

#include <chrono>
#include <iostream>
#include <string>
#include <time.h>

#define CVUI_IMPLEMENTATION
#include "cvui.h"

#define WINDOW_NAME "Capture with Kinect v2"

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
  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

  // new window
  std::cout << "Set window" << std::endl;
  cvui::init(WINDOW_NAME);
  cv::Mat cv_frame = cv::Mat(cv::Size(1164, 878), CV_8UC3);
  bool save_depth = true, save_rgb = true, save_rgb2depth = false,
       save_ir = false;

  // set opencv image
  cv::Mat rgb_cv, ir_cv, undistroted_depth_cv, rgb2depth_cv;
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

    // TODO : align depth with rgb
    if (enable_rgb && enable_depth) {
      registration->apply(rgb, depth, &undistorted, &registered);
    }

    // convert images to opencv mat
    cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgb_cv);
    cv::Mat(registered.height, registered.width, CV_8UC4, registered.data)
        .copyTo(rgb2depth_cv);
    cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(ir_cv);
    cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data)
        .copyTo(undistroted_depth_cv);

    cv::flip(rgb_cv, rgb_cv, 1);
    cv::flip(undistroted_depth_cv, undistroted_depth_cv, 1);
    cv::flip(ir_cv, ir_cv, 1);
    cv::flip(rgb2depth_cv, rgb2depth_cv, 1);

    // release the frames
    listener.release(frames);

    // gui
    cv_frame = cv::Scalar(49, 52, 49);

    if (cvui::button(cv_frame, 10, 20, 100, 30, "Save")) {
      std::chrono::time_point<std::chrono::system_clock,
                              std::chrono::milliseconds>
          tp = std::chrono::time_point_cast<std::chrono::milliseconds>(
              std::chrono::system_clock::now());
      std::time_t timestamp = tp.time_since_epoch().count();
      std::string now_time = std::to_string(timestamp);

      if (save_rgb) {
        cv::imwrite(now_time + "_RGB.png", rgb_cv);
      }
      if (save_depth) {
        cv::imwrite(now_time + "_depth.png", undistroted_depth_cv);
      }
      if (save_ir) {
        cv::imwrite(now_time + "_ir.png", ir_cv);
      }
      if (save_rgb2depth) {
        cv::imwrite(now_time + "_RGB2D.png", rgb2depth_cv);
      }
    }

    if (cvui::button(cv_frame, 10, 60, 100, 30, "Exit")) {
      protonect_shutdown = true;
    }

    cvui::checkbox(cv_frame, 10, 110, "Save depth", &save_depth);
    cvui::checkbox(cv_frame, 10, 140, "Save RGB", &save_rgb);
    cvui::checkbox(cv_frame, 10, 170, "Save ir", &save_ir);
    cvui::checkbox(cv_frame, 10, 200, "Save RGB2D", &save_rgb2depth);

    cv::Mat rgb_show, depth_show, rgb2depth_show, ir_show;
    {
      cv::cvtColor(rgb_cv, rgb_show, cv::COLOR_BGRA2BGR, 3);
      cv::resize(rgb_show, rgb_show, cv::Size(512, 424));

      cv::cvtColor(undistroted_depth_cv / 20.0f, depth_show, cv::COLOR_GRAY2BGR,
                   3);

      cv::cvtColor(rgb2depth_cv, rgb2depth_show, cv::COLOR_BGRA2BGR, 3);

      cv::cvtColor(ir_cv / 257.0f, ir_show, cv::COLOR_GRAY2BGR, 3);
    }

    cvui::image(cv_frame, 120, 10, rgb_show);
    cvui::image(cv_frame, 120, 444, depth_show);
    cvui::image(cv_frame, 642, 10, rgb2depth_show);
    cvui::image(cv_frame, 642, 444, ir_show);

    cvui::imshow(WINDOW_NAME, cv_frame);

    if (cv::waitKey(20) == 27) {
      protonect_shutdown = true;
    }
  }

  dev->stop();
  dev->close();

  return 0;
}
