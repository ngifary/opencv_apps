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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/tutorial_code/ImgTrans/HoughLines_Demo.cpp
/**
 * @file HoughLines_Demo.cpp
 * @brief Demo code for Hough Transform
 * @author OpenCV team
 */

#include "opencv_apps/hough_lines.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace opencv_apps
{
  HoughLines::HoughLines(const rclcpp::NodeOptions &options) : OpenCVNode("HoughCircles", options)
  {
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "use_camera_info";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
      desc.description = "Indicates that the camera_info topic should be subscribed to to get the default input_frame_id. Otherwise the frame from the image message will be used.";
      use_camera_info_ = declare_parameter(desc.name, false, desc);
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "hough_type";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      desc.description = "Hough Line Methods";
      desc.integer_range.resize(1);
      auto &integer_range = desc.integer_range.at(0);
      integer_range.from_value = 0;
      integer_range.to_value = 1;
      hough_type_ = declare_parameter(desc.name, 0, desc);
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "threshold";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      desc.description = "Hough Line Threshold.";
      desc.integer_range.resize(1);
      auto &integer_range = desc.integer_range.at(0);
      integer_range.from_value = 0;
      integer_range.to_value = 150;
      threshold_ = declare_parameter(desc.name, 150, desc);
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "rho";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      desc.description = "The resolution of the parameter r in pixels. We use 1 pixel.";
      desc.floating_point_range.resize(1);
      auto &floating_point_range = desc.floating_point_range.at(0);
      floating_point_range.from_value = 1.0;
      floating_point_range.to_value = 100.0;
      floating_point_range.step = 0.01;
      rho_ = declare_parameter(desc.name, 1.0, desc);
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "theta";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      desc.description = "The resolution of the parameter \theta in radians. We use 1 degree (CV_PI/180).";
      desc.floating_point_range.resize(1);
      auto &floating_point_range = desc.floating_point_range.at(0);
      floating_point_range.from_value = 1.0;
      floating_point_range.to_value = 90.0;
      floating_point_range.step = 0.01;
      theta_ = declare_parameter(desc.name, 1.0, desc);
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "minLineLength";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      desc.description = "The minimum number of points that can form a line. Lines with less than this number of points are disregarded.";
      desc.floating_point_range.resize(1);
      auto &floating_point_range = desc.floating_point_range.at(0);
      floating_point_range.from_value = 0.0;
      floating_point_range.to_value = 500.0;
      floating_point_range.step = 0.01;
      minLineLength_ = declare_parameter(desc.name, 30.0, desc);
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "maxLineGap";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      desc.description = "The maximum gap between two points to be considered in the same line.";
      desc.floating_point_range.resize(1);
      auto &floating_point_range = desc.floating_point_range.at(0);
      floating_point_range.from_value = 0.0;
      floating_point_range.to_value = 100.0;
      floating_point_range.step = 0.01;
      maxLineGap_ = declare_parameter(desc.name, 10.0, desc);
    }

    prev_stamp_ = rclcpp::Time(0, 0);

    window_name_ = "Hough Lines Demo";
    min_threshold_ = 50;
    max_threshold_ = 150;
    threshold_ = max_threshold_;

    img_pub_ = image_transport::create_publisher(this, "image", imageQoS().get_rmw_qos_profile());
    msg_pub_ = create_publisher<opencv_apps::msg::LineArrayStamped>("lines", 1);

    subscribe();
  }

  HoughLines::~HoughLines() { unsubscribe(); }

  rcl_interfaces::msg::SetParametersResult HoughLines::paramCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    // Here update class attributes, do some actions, etc.
    for (const auto &param : parameters)
    {
      if (param.get_name() == "hough_type")
      {
        hough_type_ = param.as_int();
        std::string hough_type_str_;
        switch (hough_type_)
        {
        case HoughType::STANDARD_HOUGH_TRANSFORM:
          hough_type_str_ = "Standard Hough Line";
          break;
        case HoughType::PROBABILISTIC_HOUGH_TRANSFORM:
          hough_type_str_ = "Probabilistic Hough Line";
          break;

        default:
          break;
        }
        RCLCPP_INFO(get_logger(), "[%s] New algorithm use: %s", get_name(), hough_type_str_.c_str());
      }
      if (param.get_name() == "rho")
      {
        rho_ = param.as_double();
        RCLCPP_INFO(get_logger(), "[%s] New rho: %f", get_name(), rho_);
      }
      if (param.get_name() == "theta")
      {
        theta_ = param.as_double();
        RCLCPP_INFO(get_logger(), "[%s] New theta: %f", get_name(), theta_);
      }
      if (param.get_name() == "threshold")
      {
        threshold_ = param.as_int();
        RCLCPP_INFO(get_logger(), "[%s] New threshold: %i", get_name(), threshold_);
      }
      if (param.get_name() == "minLineLength")
      {
        minLineLength_ = param.as_double();
        RCLCPP_INFO(get_logger(), "[%s] New minLineLength: %f", get_name(), minLineLength_);
      }
      if (param.get_name() == "maxLineGap")
      {
        maxLineGap_ = param.as_double();
        RCLCPP_INFO(get_logger(), "[%s] New maxLineGap: %f", get_name(), maxLineGap_);
      }
    }
    return result;
  }

  const std::string &HoughLines::frameWithDefault(const std::string &frame, const std::string &image_frame)
  {
    if (frame.empty())
      return image_frame;
    return frame;
  }

  void HoughLines::doWork(const sensor_msgs::msg::Image::ConstSharedPtr &msg, const std::string &input_frame_from_msg)
  {
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat in_image = cv_bridge::toCvShare(msg, "bgr8")->image;
      cv::Mat src_gray;

      if (in_image.channels() > 1)
      {
        cv::cvtColor(in_image, src_gray, cv::COLOR_BGR2GRAY);
        /// Apply Canny edge detector
        Canny(src_gray, in_image, 50, 200, 3);
      }
      else
      {
        /// Check whether input gray image is filtered such that canny, sobel ...etc
        bool is_filtered = true;
        for (int y = 0; y < in_image.rows; ++y)
        {
          for (int x = 0; x < in_image.cols; ++x)
          {
            if (!(in_image.at<unsigned char>(y, x) == 0 || in_image.at<unsigned char>(y, x) == 255))
            {
              is_filtered = false;
              break;
            }
            if (!is_filtered)
            {
              break;
            }
          }
        }

        if (!is_filtered)
        {
          Canny(in_image, in_image, 50, 200, 3);
        }
      }

      cv::Mat out_image;
      cv::cvtColor(in_image, out_image, CV_GRAY2BGR);

      // Messages
      opencv_apps::msg::LineArrayStamped lines_msg;
      lines_msg.header = msg->header;

      // Do the work
      std::vector<cv::Rect> faces;

      if (debug_view_)
      {
        /// Create Trackbars for Thresholds
        char thresh_label[50];
        sprintf(thresh_label, "Thres: %d + input", min_threshold_);

        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        // cv::createTrackbar(thresh_label, window_name_, &threshold_, max_threshold_, trackbarCallback);
        // if (need_config_update_)
        // {
        //   config_.threshold = threshold_;
        //   reconfigure_server_->updateConfig(config_);
        //   need_config_update_ = false;
        // }
      }

      switch (hough_type_)
      {
      case opencv_apps::STANDARD_HOUGH_TRANSFORM:
      {
        std::vector<cv::Vec2f> s_lines;

        /// 1. Use Standard Hough Transform
        cv::HoughLines(in_image, s_lines, rho_, theta_ * CV_PI / 180, threshold_, minLineLength_, maxLineGap_);

        /// Show the result
        for (const cv::Vec2f &s_line : s_lines)
        {
          float r = s_line[0], t = s_line[1];
          double cos_t = cos(t), sin_t = sin(t);
          double x0 = r * cos_t, y0 = r * sin_t;
          double alpha = 1000;

          cv::Point pt1(cvRound(x0 + alpha * (-sin_t)), cvRound(y0 + alpha * cos_t));
          cv::Point pt2(cvRound(x0 - alpha * (-sin_t)), cvRound(y0 - alpha * cos_t));

          cv::line(out_image, pt1, pt2, cv::Scalar(255, 0, 0), 3, cv::LINE_AA);

          opencv_apps::msg::Line line_msg;
          line_msg.pt1.x = pt1.x;
          line_msg.pt1.y = pt1.y;
          line_msg.pt2.x = pt2.x;
          line_msg.pt2.y = pt2.y;
          lines_msg.lines.push_back(line_msg);
        }

        break;
      }
      case opencv_apps::PROBABILISTIC_HOUGH_TRANSFORM:
      default:
      {
        std::vector<cv::Vec4i> p_lines;

        /// 2. Use Probabilistic Hough Transform
        cv::HoughLinesP(in_image, p_lines, rho_, theta_ * CV_PI / 180, threshold_, minLineLength_, maxLineGap_);

        /// Show the result
        for (const cv::Vec4i &l : p_lines)
        {
          cv::line(out_image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 0, 0), 3, cv::LINE_AA);

          opencv_apps::msg::Line line_msg;
          line_msg.pt1.x = l[0];
          line_msg.pt1.y = l[1];
          line_msg.pt2.x = l[2];
          line_msg.pt2.y = l[3];
          lines_msg.lines.push_back(line_msg);
        }

        break;
      }
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow(window_name_, out_image);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::msg::Image::Ptr out_img = cv_bridge::CvImage(msg->header, "bgr8", out_image).toImageMsg();
      img_pub_.publish(out_img);
      msg_pub_->publish(lines_msg);
    }
    catch (cv::Exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }
} // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::HoughLines)