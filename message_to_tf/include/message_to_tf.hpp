#ifndef MESSAGE_TO_TF_H
#define MESSAGE_TO_TF_H

#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2/buffer_core.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"

class MessageToTF: public rclcpp::Node{
    void odom_cb(nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    tf2::BufferCore tf_buffer_;
    tf2_ros::TransformListener transform_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    void configure_parameters();

    /// Dynamically reconfigurable parameters
    rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

    /// Parameters
    std::string odom_topic_name_, frame_id_;

public:
    MessageToTF();
    void sendTransform(geometry_msgs::msg::Pose const &pose,
                       const std_msgs::msg::Header& header,
                       std::string child_frame_id = "");
};

#endif //MESSAGE_TO_TF_H
