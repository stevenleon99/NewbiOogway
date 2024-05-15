#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace oog
{

class test_manipulatorx_ign: public rclcpp::Node {

public:
    test_manipulatorx_ign(const std::string& name);

private:
    
    std::vector<double> cur_joint = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<std::string> joint_name = {"joint1", "joint2", "joint3", "joint4", "gripper", "gripper_sub"};
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
    std::shared_ptr<rclcpp::TimerBase> timer; // periodic run publish to /joint_states
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub;

    double target_joint = 3.14;
    double stp = 0.05;
    double cur_vel = 0.0;
    double target_vel = 0.1;
    double stp_vel = 0.01;

    void _movejoint_callback();
};

} // namespace oog
