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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/tutorial_code/ImgTrans/
/**
 * @file Sobel_Demo.cpp
 * @brief Sample code using Sobel and/orScharr OpenCV functions to make a simple Edge Detector
 * @author OpenCV team
 */
/**
 * @file Laplace_Demo.cpp
 * @brief Sample code showing how to detect edges using the Laplace operator
 * @author OpenCV team
 */
/**
 * @file CannyDetector_Demo.cpp
 * @brief Sample code showing how to detect edges using the Canny Detector
 * @author OpenCV team
 */

#include "opencv_apps/edge_detection.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace opencv_apps
{
  EdgeDetection::EdgeDetection(const rclcpp::NodeOptions &options) : OpenCVNode("EdgeDetection", options)
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
      desc.name = "edge_type";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      desc.description = "Edge Detection Methods";
      desc.integer_range.resize(1);
      auto &integer_range = desc.integer_range.at(0);
      integer_range.from_value = 0;
      integer_range.to_value = 2;
      edge_type_ = declare_parameter(desc.name, 0, desc);
    }

    canny_paramps_visible_ = false;

    window_name_ = "Edge Detection Demo";
    canny_threshold1_ = 100; // only for canny
    canny_threshold2_ = 200; // only for canny

    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&EdgeDetection::paramCallback, this, std::placeholders::_1));

    img_pub_ = image_transport::create_publisher(this, "edge", imageQoS().get_rmw_qos_profile());

    RCLCPP_DEBUG(this->get_logger(), "Subscribing to image topic.");
    if (use_camera_info_)
      cam_sub_ = image_transport::create_camera_subscription(this, "image", std::bind(&EdgeDetection::imageCallbackWithInfo, this, std::placeholders::_1, std::placeholders::_2), "raw", imageQoS().get_rmw_qos_profile());
    else
      img_sub_ = image_transport::create_subscription(this, "image", std::bind(&EdgeDetection::imageCallback, this, std::placeholders::_1), "raw", imageQoS().get_rmw_qos_profile());
  }

  EdgeDetection::~EdgeDetection() {}

  void EdgeDetection::undeclareCannyParameter()
  {
    std::vector<std::string> param_names = {"canny_threshold1", "canny_threshold2", "apertureSize", "apply_blur_pre", "postBlurSize", "postBlurSigma", "apply_blur_post", "L2gradient"};
    std::vector<rclcpp::Parameter> params = get_parameters(param_names);
    for (auto param_name : param_names)
    {
      undeclare_parameter(param_name);
    }
    RCLCPP_INFO(get_logger(), "[EdgeDetection] Canny Parameters are now deleted");
  }

  void EdgeDetection::declareCannyParameter()
  {
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "canny_threshold1";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      desc.description = "First threshold for the hysteresis procedure.";
      desc.dynamic_typing = true;
      desc.integer_range.resize(1);
      auto &integer_range = desc.integer_range.at(0);
      integer_range.from_value = 0;
      integer_range.to_value = 500;
      canny_threshold1_ = declare_parameter(desc.name, 100, desc);
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "canny_threshold2";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      desc.description = "Second threshold for the hysteresis procedure.";
      desc.dynamic_typing = true;
      desc.integer_range.resize(1);
      auto &integer_range = desc.integer_range.at(0);
      integer_range.from_value = 0;
      integer_range.to_value = 500;
      canny_threshold2_ = declare_parameter(desc.name, 200, desc);
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "apertureSize";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      desc.description = "Aperture size for the Sobel() operator.";
      desc.dynamic_typing = true;
      desc.integer_range.resize(1);
      auto &integer_range = desc.integer_range.at(0);
      integer_range.from_value = 1;
      integer_range.to_value = 10;
      apertureSize_ = declare_parameter(desc.name, 3, desc);
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "apply_blur_pre";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
      desc.description = "Flag, applying Blur() to input image";
      desc.dynamic_typing = true;
      apply_blur_pre_ = declare_parameter(desc.name, true, desc);
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "postBlurSize";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      desc.description = "Aperture size for the Blur() operator.";
      desc.dynamic_typing = true;
      desc.integer_range.resize(1);
      auto &integer_range = desc.integer_range.at(0);
      integer_range.from_value = 3;
      integer_range.to_value = 31;
      postBlurSize_ = declare_parameter(desc.name, 13, desc);
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "postBlurSigma";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      desc.description = "Sigma for the GaussianBlur() operator.";
      desc.dynamic_typing = true;
      desc.floating_point_range.resize(1);
      auto &floating_point_range = desc.floating_point_range.at(0);
      floating_point_range.from_value = 0.0;
      floating_point_range.to_value = 10.0;
      postBlurSigma_ = declare_parameter(desc.name, 3.2, desc);
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "apply_blur_post";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
      desc.description = "Flag, applying GaussianBlur() to output(edge) image";
      desc.dynamic_typing = true;
      apply_blur_post_ = declare_parameter(desc.name, false, desc);
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "L2gradient";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
      desc.description = "Flag, indicating whether a more accurate  L_2 norm should be used to calculate the image gradient magnitude ( L2gradient=true ), or whether the default  L_1 norm is enough ( L2gradient=false ).";
      desc.dynamic_typing = true;
      L2gradient_ = declare_parameter(desc.name, false, desc);
    }
    RCLCPP_INFO(get_logger(), "[EdgeDetection] Canny Parameters can now be set dynamically");
  }

  rcl_interfaces::msg::SetParametersResult EdgeDetection::paramCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    // Here update class attributes, do some actions, etc.
    for (const auto &param : parameters)
    {
      if (param.get_name() == "edge_type")
      {
        edge_type_ = param.as_int();
        std::string edge_type_str_;
        switch (edge_type_)
        {
        case EdgeType::SOBEL:
          edge_type_str_ = "Sobel";
          break;
        case EdgeType::LAPLACE:
          edge_type_str_ = "Laplace";
          break;
        case EdgeType::CANNY:
          edge_type_str_ = "Canny";
          break;

        default:
          break;
        }
        RCLCPP_INFO(get_logger(), "[%s] New algorithm use: %s", get_name(), edge_type_str_.c_str());
      }
      if (param.get_name() == "canny_threshold1")
      {
        canny_threshold1_ = param.as_int();
        RCLCPP_INFO(get_logger(), "[%s] New canny_threshold1: %i", get_name(), canny_threshold1_);
      }
      if (param.get_name() == "canny_threshold2")
      {
        canny_threshold2_ = param.as_int();
        RCLCPP_INFO(get_logger(), "[%s] New canny_threshold2: %i", get_name(), canny_threshold2_);
      }
      if (param.get_name() == "apertureSize")
      {
        apertureSize_ = param.as_int();
        RCLCPP_INFO(get_logger(), "[%s] New apertureSize: %i", get_name(), apertureSize_);
      }
      if (param.get_name() == "L2gradient")
      {
        L2gradient_ = param.as_bool();
        RCLCPP_INFO(get_logger(), "[%s] Setting the L2gradient value to: %s.", get_name(), L2gradient_ ? "true" : "false");
      }
      if (param.get_name() == "apply_blur_pre")
      {
        apply_blur_pre_ = param.as_bool();
        RCLCPP_INFO(get_logger(), "[%s] Setting the apply_blur_pre value to: %s.", get_name(), apply_blur_pre_ ? "true" : "false");
      }
      if (param.get_name() == "apply_blur_post")
      {
        apply_blur_post_ = param.as_bool();
        RCLCPP_INFO(get_logger(), "[%s] Setting the apply_blur_post value to: %s.", get_name(), apply_blur_post_ ? "true" : "false");
      }
      if (param.get_name() == "postBlurSize")
      {
        postBlurSize_ = param.as_int();
        RCLCPP_INFO(get_logger(), "[%s] New postBlurSize: %i", get_name(), postBlurSize_);
      }
      if (param.get_name() == "postBlurSigma")
      {
        postBlurSigma_ = param.as_double();
        RCLCPP_INFO(get_logger(), "[%s] New postBlurSigma: %f", get_name(), postBlurSigma_);
      }
    }
    return result;
  }

  const std::string &EdgeDetection::frameWithDefault(const std::string &frame, const std::string &image_frame)
  {
    if (frame.empty())
      return image_frame;
    return frame;
  }

  void EdgeDetection::imageCallbackWithInfo(const sensor_msgs::msg::Image::ConstSharedPtr &msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cam_info)
  {
    doWork(msg, cam_info->header.frame_id);
  }

  void EdgeDetection::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    doWork(msg, msg->header.frame_id);
  }

  void EdgeDetection::doWork(const sensor_msgs::msg::Image::ConstSharedPtr &msg, const std::string &input_frame_from_msg)
  {
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      // Do the work
      cv::Mat src_gray;
      cv::GaussianBlur(frame, frame, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);

      /// Convert it to gray
      if (frame.channels() > 1)
      {
        cv::cvtColor(frame, src_gray, cv::COLOR_RGB2GRAY);
      }
      else
      {
        src_gray = frame;
      }

      /// Create window
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
      }

      std::string new_window_name;
      cv::Mat grad;
      switch (edge_type_)
      {
      case opencv_apps::EdgeType::SOBEL:
      {
        if (canny_paramps_visible_)
        {
          undeclareCannyParameter();
          canny_paramps_visible_ = false;
        }
        /// Generate grad_x and grad_y
        cv::Mat grad_x, grad_y;
        cv::Mat abs_grad_x, abs_grad_y;

        int scale = 1;
        int delta = 0;
        int ddepth = CV_16S;

        /// Gradient X
        // Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
        cv::Sobel(src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
        cv::convertScaleAbs(grad_x, abs_grad_x);

        /// Gradient Y
        // Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
        cv::Sobel(src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
        cv::convertScaleAbs(grad_y, abs_grad_y);

        /// Total Gradient (approximate)
        cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

        new_window_name = "Sobel Edge Detection Demo";
        break;
      }
      case opencv_apps::EdgeType::LAPLACE:
      {
        if (canny_paramps_visible_)
        {
          undeclareCannyParameter();
          canny_paramps_visible_ = false;
        }
        cv::Mat dst;
        int kernel_size = 3;
        int scale = 1;
        int delta = 0;
        int ddepth = CV_16S;
        /// Apply Laplace function

        cv::Laplacian(src_gray, dst, ddepth, kernel_size, scale, delta, cv::BORDER_DEFAULT);
        convertScaleAbs(dst, grad);

        new_window_name = "Laplace Edge Detection Demo";
        break;
      }
      case opencv_apps::EdgeType::CANNY:
      {
        if (!canny_paramps_visible_)
        {
          declareCannyParameter();
          canny_paramps_visible_ = true;
        }
        int edge_thresh = 1;
        int kernel_size = 3;
        int const max_canny_threshold1 = 500;
        int const max_canny_threshold2 = 500;
        cv::Mat detected_edges;

        /// Reduce noise with a kernel 3x3
        if (apply_blur_pre_)
        {
          cv::blur(src_gray, src_gray, cv::Size(apertureSize_, apertureSize_));
        }

        /// Canny detector
        cv::Canny(src_gray, grad, canny_threshold1_, canny_threshold2_, kernel_size, L2gradient_);
        if (apply_blur_post_)
        {
          cv::GaussianBlur(grad, grad, cv::Size(postBlurSize_, postBlurSize_), postBlurSigma_,
                           postBlurSigma_); // 0.3*(ksize/2 - 1) + 0.8
        }

        new_window_name = "Canny Edge Detection Demo";

        // TODO: Create a Trackbar for user to enter threshold
        // if (debug_view_)
        // {
        //   if (window_name_ == new_window_name)
        //   {
        //     cv::createTrackbar("Min CannyThreshold1:", window_name_, nullptr, max_canny_threshold1);
        //     cv::setTrackbarPos("Min CannyThreshold1:", window_name_, canny_threshold1_);
        //     cv::createTrackbar("Min CannyThreshold2:", window_name_, nullptr, max_canny_threshold2);
        //     cv::setTrackbarPos("Min CannyThreshold2:", window_name_, canny_threshold2_);
        //   }
        // }
        break;
      }
      }

      if (debug_view_)
      {
        if (window_name_ != new_window_name)
        {
          cv::destroyWindow(window_name_);
          window_name_ = new_window_name;
        }
        cv::imshow(window_name_, grad);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::msg::Image::SharedPtr out_img =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, grad).toImageMsg();
      out_img->header.frame_id = input_frame_from_msg;
      img_pub_.publish(out_img);
    }
    catch (cv::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }

} // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::EdgeDetection)