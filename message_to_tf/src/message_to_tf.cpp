#include "message_to_tf.hpp"

MessageToTF::MessageToTF():
rclcpp::Node("message_to_tf"),
transform_listener_(tf_buffer_),
tf_broadcaster_(this)
{
    configure_parameters();
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_name_, rclcpp::SensorDataQoS(), std::bind(&MessageToTF::odom_cb, this, std::placeholders::_1));}

void MessageToTF::sendTransform(geometry_msgs::msg::Pose const &pose, const std_msgs::msg::Header& header, std::string child_frame_id){
    // This will send a transform between the frame in the header (target frame) and the frame_id_ (desired source frame)
    // if the frame_id_ is the same as  the child_frame_id then
    // we do not need to remove the intermediate trasnform from the overall transform before publishing

    // Convert the pose message to tf2 transform object for easier manipulation
    tf2::Transform target_to_frame;
    tf2::fromMsg(pose, target_to_frame);

    // Find intermediate transform if frame_id_ does not equal child frame
    if(child_frame_id != frame_id_){
      while(!tf_buffer_.canTransform(frame_id_, child_frame_id, tf2::TimePointZero)){usleep(1e3);}
      auto base_to_odom_msg = tf_buffer_.lookupTransform(frame_id_, child_frame_id, tf2::TimePointZero);
      tf2::Stamped<tf2::Transform> base_to_odom;
      tf2::fromMsg(base_to_odom_msg, base_to_odom);
      target_to_frame = target_to_frame * base_to_odom.inverse();
    }

    geometry_msgs::msg::Pose target_to_frame_pose;
    tf2::toMsg(target_to_frame, target_to_frame_pose);

    geometry_msgs::msg::TransformStamped transform_stamped_msg;
    transform_stamped_msg.child_frame_id = frame_id_;
    transform_stamped_msg.header = header;
    transform_stamped_msg.transform.translation.x = target_to_frame_pose.position.x;
    transform_stamped_msg.transform.translation.y = target_to_frame_pose.position.y;
    transform_stamped_msg.transform.translation.z = target_to_frame_pose.position.z;
    transform_stamped_msg.transform.rotation = target_to_frame_pose.orientation;
    tf_broadcaster_.sendTransform(transform_stamped_msg);
}

void MessageToTF::odom_cb(nav_msgs::msg::Odometry::SharedPtr msg) {
      sendTransform(msg->pose.pose, msg->header, msg->child_frame_id);
}


void MessageToTF::configure_parameters() {
    odom_topic_name_ = this->declare_parameter("odom_topic_name", "/odom");
    frame_id_ = this->declare_parameter("frame_id", "base_link");

    /// Update parameters dynamically
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
    auto on_parameter_event_callback = [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
        std::stringstream ss;
        ss << "\nParameter event:\n changed parameters:";
        for (auto & changed_parameter : event->changed_parameters) {
            ss << "\n  " << changed_parameter.name;
            if(changed_parameter.name == "frame_id") {
                frame_id_ = changed_parameter.value.string_value;
            }
        }
        ss << "\n";
        RCLCPP_DEBUG(this->get_logger(), ss.str().c_str());
    };

    /// Setup callback for changes to parameters.
    parameter_event_sub_ = parameters_client_->on_parameter_event(on_parameter_event_callback);
}
