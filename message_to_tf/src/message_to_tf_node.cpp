#include "message_to_tf.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MessageToTF>());
    rclcpp::shutdown();
    return 0;
}
