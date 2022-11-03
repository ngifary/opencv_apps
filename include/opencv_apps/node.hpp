#ifndef OPENCV_APPS_NODE_HPP_
#define OPENCV_APPS_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/simple_publisher_plugin.hpp"
#include "image_transport/simple_subscriber_plugin.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgproc.hpp"

namespace opencv_apps
{
    class OpenCVNode : public rclcpp::Node
    {
    private:
    protected:
        /** \brief The maximum queue size (default: 3). */
        int max_queue_size_ = 3;

        /** \brief The output Image publisher. */
        image_transport::Publisher img_pub_;

        /** \brief The output Camera publisher. */
        image_transport::CameraPublisher cam_pub_;

        /** \brief The input Image subscriber. */
        image_transport::Subscriber img_sub_;

        /** \brief The input Camera subscriber. */
        image_transport::CameraSubscriber cam_sub_;

        /** \brief Indicates that the camera_info topic should be subscribed to */
        bool use_camera_info_;

        /** \brief display output on GUI. */
        bool debug_view_ = false;

        rclcpp::Time prev_stamp_ = rclcpp::Time(0, 0);

        OnSetParametersCallbackHandle::SharedPtr config_callback_handle_;

        rcl_interfaces::msg::SetParametersResult configCallback(const std::vector<rclcpp::Parameter> &parameters);

        /** \brief Return QoS settings for image topic */
        rclcpp::QoS
        imageQoS() const
        {
            rclcpp::QoS qos(max_queue_size_);
            return qos;
        }

        void imageCallbackWithInfo(const sensor_msgs::msg::Image::ConstSharedPtr &msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cam_info);

        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

        /**
         * @brief Virtual abstract doWork method. To be implemented by every child. 
         * 
         * @param msg 
         * @param input_frame_from_msg 
         */
        virtual void 
        doWork(const sensor_msgs::msg::Image::ConstSharedPtr &msg, const std::string &input_frame_from_msg);

        void subscribe();

        void unsubscribe();

    public:
        OpenCVNode(std::string node_name, const rclcpp::NodeOptions &options);
        ~OpenCVNode();
    };

} // namespace opencv_apps

#endif // OPENCV_APPS_NODE_HPP_