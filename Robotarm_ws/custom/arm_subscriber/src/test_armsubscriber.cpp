#include "rclcpp/rclcpp.hpp"
#include "arm_subscriber/arm_subscriber.hpp"


int main(int argc, char const *argv[])
{

    
    rclcpp::init(argc, argv);
    auto node_joint = std::make_shared<oog::Status_Subscriber>("statusubcriber");
    
    if (rclcpp::ok()){
        rclcpp::spin(node_joint);
    }

    rclcpp::shutdown();

    
    return 0;
}
