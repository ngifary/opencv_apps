#include "opencv_apps/node.hpp"

namespace opencv_apps
{
    OpenCVNode::OpenCVNode(std::string node_name, const rclcpp::NodeOptions &options) : rclcpp::Node(node_name, options)
    {
        // {
        //     rcl_interfaces::msg::ParameterDescriptor desc;
        //     desc.name = "latch";
        //     desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
        //     desc.description = "set true if latch topic publication.";
        //     latch_ = declare_parameter(desc.name, latch_, desc);
        // }
        {
            rcl_interfaces::msg::ParameterDescriptor desc;
            desc.name = "debug_view";
            desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
            desc.description = "set true to adjust parameter at runtime.";
            debug_view_ = declare_parameter(desc.name, debug_view_, desc);
        }
        {
            rcl_interfaces::msg::ParameterDescriptor desc;
            desc.name = "max_queue_size";
            desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
            desc.description = "QoS History depth";
            desc.read_only = true;
            max_queue_size_ = declare_parameter(desc.name, max_queue_size_, desc);
        }

        RCLCPP_DEBUG(this->get_logger(), "OpenCV Node successfully created with the following parameters:\n"
                                        //  " - latch            : %s\n"
                                         " - debug_view       : %s\n"
                                         " - max_queue_size   : %i",
                    //  (latch_) ? "true" : "false",
                     (debug_view_) ? "true" : "false",
                     max_queue_size_);

        config_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&OpenCVNode::configCallback, this, std::placeholders::_1));
    }

    OpenCVNode::~OpenCVNode() {}

    void OpenCVNode::unsubscribe()
    {
        RCLCPP_DEBUG(this->get_logger(), "Unsubscribing from image topic.");
        img_sub_.shutdown();
        cam_sub_.shutdown();
        img_pub_.shutdown();
        cam_pub_.shutdown();
    }

    rcl_interfaces::msg::SetParametersResult OpenCVNode::configCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        // Here update class attributes, do some actions, etc.
        for (const auto &param : parameters)
        {
            // if (param.get_name() == "latch")
            // {
            //     latch_ = param.as_bool();
            //     RCLCPP_INFO(get_logger(), "[Node] Setting the latch value to: %s.", latch_ ? "true" : "false");
            // }
            if (param.get_name() == "debug_view")
            {
                debug_view_ = param.as_bool();
                RCLCPP_INFO(get_logger(), "[Node] Setting the debug_view value to: %s.", debug_view_ ? "true" : "false");
            }
        }
        return result;
    }

} // namespace opencv_apps
