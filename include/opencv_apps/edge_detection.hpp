#ifndef OPENCV_APPS_EDGE_DETECTION_NODE_HPP_
#define OPENCV_APPS_EDGE_DETECTION_NODE_HPP_

#include "opencv2/highgui/highgui.hpp"

#include "opencv_apps/node.hpp"

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
        // /** \brief The maximum queue size (default: 3). */
        // int max_queue_size_ = 3;

        // /** \brief display output on GUI. */
        // bool debug_view_ = false;

        /** \brief edge detection parameters */
        bool canny_paramps_visible_;
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

        void doWork(const sensor_msgs::msg::Image::ConstSharedPtr &msg, const std::string &input_frame_from_msg);

        void undeclareCannyParameter();

        void declareCannyParameter();

    public:
        EdgeDetection(const rclcpp::NodeOptions &options);
        ~EdgeDetection();
    };

} // namespace opencv_apps

#endif // OPENCV_APPS_EDGE_DETECTION_NODE_HPP_