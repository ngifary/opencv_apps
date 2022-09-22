// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Kei Okada.
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
 *   * Neither the name of the Kei Okada nor the names of its
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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/tutorial_code/ImgTrans/HoughCircle_Demo.cpp
/**
 * @file HoughCircle_Demo.cpp
 * @brief Demo code for Hough Transform
 * @author OpenCV team
 */

#include "opencv_apps/hough_circles.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace opencv_apps
{
  HoughCircles::HoughCircles(const rclcpp::NodeOptions &options) : OpenCVNode("HoughCircles", options)
  {
    msg_pub_ = create_publisher<opencv_apps::msg::CircleArrayStamped>("circles", 1);
    // Nodelet::onInit();
    // it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(*nh_));

    // debug_image_type_ = 0;
    // pnh_->param("queue_size", queue_size_, 3);
    // pnh_->param("debug_view", debug_view_, false);
    // if (debug_view_)
    // {
    //   always_subscribe_ = debug_view_;
    // }
    // prev_stamp_ = ros::Time(0, 0);

    // window_name_ = "Hough Circle Detection Demo";
    // canny_threshold_initial_value_ = 200;
    // accumulator_threshold_initial_value_ = 50;
    // max_accumulator_threshold_ = 200;
    // max_canny_threshold_ = 255;
    // min_distance_between_circles_ = 0;

    // // declare and initialize both parameters that are subjects to change
    // canny_threshold_ = canny_threshold_initial_value_;
    // accumulator_threshold_ = accumulator_threshold_initial_value_;

    // reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config>>(*pnh_);
    // dynamic_reconfigure::Server<Config>::CallbackType f =
    //     boost::bind(&HoughCirclesNodelet::reconfigureCallback, this, boost::placeholders::_1, boost::placeholders::_2);
    // reconfigure_server_->setCallback(f);

    // img_pub_ = advertiseImage(*pnh_, "image", 1);
    // msg_pub_ = advertise<opencv_apps::msg::CircleArrayStamped>(*pnh_, "circles", 1);

    // debug_image_type_ = 0;
    // debug_image_pub_ = advertiseImage(*pnh_, "debug_image", 1);

    // onInitPostProcess();
  }

  HoughCircles::~HoughCircles()
  {
  }

  rcl_interfaces::msg::SetParametersResult HoughCircles::paramCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    // Here update class attributes, do some actions, etc.
    for (const auto &param : parameters)
    {
      if (param.get_name() == "debug_image_type")
      {
        debug_image_type_ = param.as_int();
        std::string debug_image_type_str_;
        switch (debug_image_type_)
        {
        case DebugViewType::INPUT:
          debug_image_type_str_ = "Input";
          break;
        case DebugViewType::BLUR:
          debug_image_type_str_ = "Blur";
          break;
        case DebugViewType::CANNY:
          debug_image_type_str_ = "Canny";
          break;

        default:
          break;
        }
        RCLCPP_INFO(get_logger(), "[%s] New algorithm use: %s", get_name(), debug_image_type_str_.c_str());
      }
      if (param.get_name() == "canny_threshold")
      {
        canny_threshold_ = param.as_double();
        RCLCPP_INFO(get_logger(), "[%s] New canny_threshold: %f", get_name(), canny_threshold_);
      }
      if (param.get_name() == "accumulator_threshold")
      {
        accumulator_threshold_ = param.as_double();
        RCLCPP_INFO(get_logger(), "[%s] New accumulator_threshold: %f", get_name(), accumulator_threshold_);
      }
      if (param.get_name() == "gaussian_blur_size")
      {
        gaussian_blur_size_ = param.as_int();
        RCLCPP_INFO(get_logger(), "[%s] New gaussian_blur_size: %i", get_name(), gaussian_blur_size_);
      }
      if (param.get_name() == "gaussian_sigma_x")
      {
        gaussian_sigma_x_ = param.as_double();
        RCLCPP_INFO(get_logger(), "[%s] New gaussian_sigma_x: %f", get_name(), gaussian_sigma_x_);
      }
      if (param.get_name() == "gaussian_sigma_y")
      {
        gaussian_sigma_y_ = param.as_double();
        RCLCPP_INFO(get_logger(), "[%s] New gaussian_sigma_y: %f", get_name(), gaussian_sigma_y_);
      }
      if (param.get_name() == "dp")
      {
        dp_ = param.as_double();
        RCLCPP_INFO(get_logger(), "[%s] New dp: %f", get_name(), dp_);
      }
      if (param.get_name() == "min_circle_radius")
      {
        min_circle_radius_ = param.as_int();
        RCLCPP_INFO(get_logger(), "[%s] New min_circle_radius: %i", get_name(), min_circle_radius_);
      }
      if (param.get_name() == "max_circle_radius")
      {
        max_circle_radius_ = param.as_int();
        RCLCPP_INFO(get_logger(), "[%s] New max_circle_radius: %i", get_name(), max_circle_radius_);
      }
      if (param.get_name() == "min_distance_between_circles")
      {
        min_distance_between_circles_ = param.as_double();
        RCLCPP_INFO(get_logger(), "[%s] New min_distance_between_circles: %f", get_name(), min_distance_between_circles_);
      }
    }
    canny_threshold_int = int(canny_threshold_);
    accumulator_threshold_int = int(accumulator_threshold_);
    gaussian_sigma_x_int = int(gaussian_sigma_x_);
    gaussian_sigma_y_int = int(gaussian_sigma_y_);
    min_distance_between_circles_int = int(min_distance_between_circles_);
    dp_int = int(dp_);

    return result;
  }

  const std::string &HoughCircles::frameWithDefault(const std::string &frame, const std::string &image_frame)
  {
    if (frame.empty())
      return image_frame;
    return frame;
  }

  void HoughCircles::imageCallbackWithInfo(const sensor_msgs::msg::Image::ConstSharedPtr &msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cam_info)
  {
    doWork(msg, cam_info->header.frame_id);
  }

  void HoughCircles::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    doWork(msg, msg->header.frame_id);
  }
  
  void HoughCircles::doWork(const sensor_msgs::msg::Image::ConstSharedPtr &msg, const std::string &input_frame_from_msg)
  {
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

      // Messages
      opencv_apps::msg::CircleArrayStamped circles_msg;
      circles_msg.header = msg->header;

      // Do the work
      std::vector<cv::Rect> faces;
      cv::Mat src_gray, edges;

      if (frame.channels() > 1)
      {
        cv::cvtColor(frame, src_gray, cv::COLOR_BGR2GRAY);
      }
      else
      {
        src_gray = frame;
      }

      // create the main window, and attach the trackbars
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);

        // cv::createTrackbar("Canny Threshold", window_name_, &canny_threshold_int, max_canny_threshold_,
        //                    trackbarCallback);
        // cv::createTrackbar("Accumulator Threshold", window_name_, &accumulator_threshold_int,
        //                    max_accumulator_threshold_, trackbarCallback);
        // cv::createTrackbar("Gaussian Blur Size", window_name_, &gaussian_blur_size_, 30, trackbarCallback);
        // cv::createTrackbar("Gaussian Sigam X", window_name_, &gaussian_sigma_x_int, 10, trackbarCallback);
        // cv::createTrackbar("Gaussian Sigma Y", window_name_, &gaussian_sigma_y_int, 10, trackbarCallback);
        // cv::createTrackbar("Min Distance between Circles", window_name_, &min_distance_between_circles_int, 100,
        //                    trackbarCallback);
        // cv::createTrackbar("Dp", window_name_, &dp_int, 100, trackbarCallback);
        // cv::createTrackbar("Min Circle Radius", window_name_, &min_circle_radius_, 500, trackbarCallback);
        // cv::createTrackbar("Max Circle Radius", window_name_, &max_circle_radius_, 2000, trackbarCallback);

        // if (need_config_update_)
        // {
        //   config_.canny_threshold = canny_threshold_ = (double)canny_threshold_int;
        //   config_.accumulator_threshold = accumulator_threshold_ = (double)accumulator_threshold_int;
        //   config_.gaussian_blur_size = gaussian_blur_size_;
        //   config_.gaussian_sigma_x = gaussian_sigma_x_ = (double)gaussian_sigma_x_int;
        //   config_.gaussian_sigma_y = gaussian_sigma_y_ = (double)gaussian_sigma_y_int;
        //   config_.min_distance_between_circles = min_distance_between_circles_ =
        //       (double)min_distance_between_circles_int;
        //   config_.dp = dp_ = (double)dp_int;
        //   config_.min_circle_radius = min_circle_radius_;
        //   config_.max_circle_radius = max_circle_radius_;
        //   reconfigure_server_->updateConfig(config_);
        //   need_config_update_ = false;
        // }
      }

      // Reduce the noise so we avoid false circle detection
      // gaussian_blur_size_ must be odd number
      if (gaussian_blur_size_ % 2 != 1)
      {
        gaussian_blur_size_ = gaussian_blur_size_ + 1;
      }
      cv::GaussianBlur(src_gray, src_gray, cv::Size(gaussian_blur_size_, gaussian_blur_size_), gaussian_sigma_x_,
                       gaussian_sigma_y_);

      // those paramaters cannot be =0
      // so we must check here
      canny_threshold_ = std::max(canny_threshold_, 1.0);
      accumulator_threshold_ = std::max(accumulator_threshold_, 1.0);

      if (debug_view_)
      {
        // https://github.com/Itseez/opencv/blob/2.4.8/modules/imgproc/src/hough.cpp#L817
        cv::Canny(frame, edges, MAX(canny_threshold_ / 2, 1), canny_threshold_, 3);
      }
      if (min_distance_between_circles_ == 0)
      { // set inital value
        min_distance_between_circles_ = src_gray.rows / 8;
      }
      // runs the detection, and update the display
      // will hold the results of the detection
      std::vector<cv::Vec3f> circles;
      // runs the actual detection
      cv::HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, dp_, min_distance_between_circles_, canny_threshold_,
                       accumulator_threshold_, min_circle_radius_, max_circle_radius_);

      cv::Mat out_image;
      if (frame.channels() == 1)
      {
        cv::cvtColor(frame, out_image, cv::COLOR_GRAY2BGR);
      }
      else
      {
        out_image = frame;
      }

      // clone the colour, input image for displaying purposes
      for (const cv::Vec3f &i : circles)
      {
        cv::Point center(cvRound(i[0]), cvRound(i[1]));
        int radius = cvRound(i[2]);
        // circle center
        circle(out_image, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
        // circle outline
        circle(out_image, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);

        opencv_apps::msg::Circle circle_msg;
        circle_msg.center.x = center.x;
        circle_msg.center.y = center.y;
        circle_msg.radius = radius;
        circles_msg.circles.push_back(circle_msg);
      }

      // shows the results
      if (debug_view_ || debug_image_pub_.getNumSubscribers() > 0)
      {
        cv::Mat debug_image;
        switch (debug_image_type_)
        {
        case 1:
          debug_image = src_gray;
          break;
        case 2:
          debug_image = edges;
          break;
        default:
          debug_image = frame;
          break;
        }
        if (debug_view_)
        {
          cv::imshow(window_name_, debug_image);
          int c = cv::waitKey(1);
          if (c == 's')
          {
            debug_image_type_ = (++debug_image_type_) % 3;
          }
        }
        if (debug_image_pub_.getNumSubscribers() > 0)
        {
          sensor_msgs::msg::Image::SharedPtr out_debug_img =
              cv_bridge::CvImage(msg->header, msg->encoding, debug_image).toImageMsg();
          debug_image_pub_.publish(out_debug_img);
        }
      }

      // Publish the image.
      sensor_msgs::msg::Image::SharedPtr out_img = cv_bridge::CvImage(msg->header, "bgr8", out_image).toImageMsg();
      img_pub_.publish(out_img);
      msg_pub_->publish(circles_msg);
    }
    catch (cv::Exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }

  // void subscribe() // NOLINT(modernize-use-override)
  // {
  //   RCLCPP_DEBUG(get_loggger(), "Subscribing to image topic.");
  //   if (config_.use_camera_info)
  //     cam_sub_ = it_->subscribeCamera("image", queue_size_, &HoughCirclesNodelet::imageCallbackWithInfo, this);
  //   else
  //     img_sub_ = it_->subscribe("image", queue_size_, &HoughCirclesNodelet::imageCallback, this);
  // }
} // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::HoughCircles)
