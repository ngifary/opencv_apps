#ifndef OPENCV_APPS_EDGE_DETECTION_NODE_HPP_
#define OPENCV_APPS_EDGE_DETECTION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv_apps/node.hpp>

namespace opencv_apps
{
    enum EdgeType
    {
        SOBEL,
        LAPLACE,
        CANNY
    };
    class EdgeDetection : public OpenCVNode
    {
    private:
        /* data */

        /** \brief edge detection parameters */
        int edge_type_;
        int canny_threshold1_;
        int canny_threshold2_;
        int apertureSize_;
        bool L2gradient_;
        bool apply_blur_pre_;
        bool apply_blur_post_;
        int postBlurSize_;
        double postBlurSigma_;

        std::string window_name_;

        OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

        rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> &parameters);

        const std::string &frameWithDefault(const std::string &frame, const std::string &image_frame);

        void imageCallbackWithInfo(const sensor_msgs::msg::Image::ConstSharedPtr &msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cam_info);

        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

        void doWork(const sensor_msgs::msg::Image::ConstSharedPtr &msg, const std::string &input_frame_from_msg);

        void declareCannyParameter();

    public:
        EdgeDetection(const rclcpp::NodeOptions &options);
        ~EdgeDetection();
    };

} // namespace opencv_apps

#endif