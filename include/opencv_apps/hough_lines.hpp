#ifndef OPENCV_APPS_HOUGH_LINES_NODE_HPP_
#define OPENCV_APPS_HOUGH_LINES_NODE_HPP_

#include "opencv2/highgui/highgui.hpp"
#include "opencv_apps/msg/line.h"
#include "opencv_apps/msg/line_array.h"
#include "opencv_apps/msg/line_array_stamped.h"

#include "opencv_apps/node.hpp"

namespace opencv_apps
{
    enum HoughType
    {
        STANDARD_HOUGH_TRANSFORM,
        PROBABILISTIC_HOUGH_TRANSFORM
    };
    class HoughLines : public OpenCVNode
    {
    private:
        /* data */
        // image_transport::Publisher img_pub_;
        // image_transport::Subscriber img_sub_;
        // image_transport::CameraSubscriber cam_sub_;
        std::shared_ptr<rclcpp::Publisher<opencv_apps::msg::LineArrayStamped>> msg_pub_;

        std::string window_name_;

        int min_threshold_;
        int max_threshold_;
        int threshold_;
        double rho_;
        double theta_;
        double minLineLength_;
        double maxLineGap_;

        int hough_type_;

        OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

        rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> &parameters);

        const std::string &frameWithDefault(const std::string &frame, const std::string &image_frame);

        void imageCallbackWithInfo(const sensor_msgs::msg::Image::ConstSharedPtr &msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cam_info);

        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

        void doWork(const sensor_msgs::msg::Image::ConstSharedPtr &msg, const std::string &input_frame_from_msg);

    public:
        HoughLines(const rclcpp::NodeOptions &options);
        ~HoughLines();
    };

} // namespace opencv_apps

#endif