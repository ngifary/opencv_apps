#ifndef OPENCV_APPS_HOUGH_CIRCLES_NODE_HPP_
#define OPENCV_APPS_HOUGH_CIRCLES_NODE_HPP_

#include "opencv2/highgui/highgui.hpp"
// #include "opencv_apps/HoughCirclesConfig.h"
#include "opencv_apps/msg/circle.hpp"
#include "opencv_apps/msg/circle_array.hpp"
#include "opencv_apps/msg/circle_array_stamped.hpp"

#include "opencv_apps/node.hpp"

namespace opencv_apps
{
    enum DebugViewType
    {
        INPUT,
        BLUR,
        CANNY
    };
    class HoughCircles : public OpenCVNode
    {
    private:
        /* data */
        // image_transport::Publisher img_pub_;
        // image_transport::Subscriber img_sub_;
        // image_transport::CameraSubscriber cam_sub_;
        std::shared_ptr<rclcpp::Publisher<opencv_apps::msg::CircleArrayStamped>> msg_pub_;

        // boost::shared_ptr<image_transport::ImageTransport> it_;

        // typedef opencv_apps::HoughCirclesConfig Config;
        // typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
        // Config config_;
        // boost::shared_ptr<ReconfigureServer> reconfigure_server_;

        // int queue_size_;
        // bool debug_view_;
        // ros::Time prev_stamp_;

        std::string window_name_;
        // static bool need_config_update_;

        // initial and max values of the parameters of interests.
        int canny_threshold_initial_value_;
        int accumulator_threshold_initial_value_;
        int max_accumulator_threshold_;
        int max_canny_threshold_;
        double canny_threshold_;
        int canny_threshold_int; // for trackbar
        double accumulator_threshold_;
        int accumulator_threshold_int;
        int gaussian_blur_size_;
        double gaussian_sigma_x_;
        int gaussian_sigma_x_int;
        double gaussian_sigma_y_;
        int gaussian_sigma_y_int;
        int voting_threshold_;
        double min_distance_between_circles_;
        int min_distance_between_circles_int;
        double dp_;
        int dp_int;
        int min_circle_radius_;
        int max_circle_radius_;

        image_transport::Publisher debug_image_pub_;
        int debug_image_type_;

        OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

        rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> &parameters);

        const std::string &frameWithDefault(const std::string &frame, const std::string &image_frame);

        void imageCallbackWithInfo(const sensor_msgs::msg::Image::ConstSharedPtr &msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cam_info);

        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

        void doWork(const sensor_msgs::msg::Image::ConstSharedPtr &msg, const std::string &input_frame_from_msg);

    public:
        HoughCircles(const rclcpp::NodeOptions &options);
        ~HoughCircles();
    };

} // namespace opencv_apps

#endif