#include "rclcpp/rclcpp.hpp"
#include "open_manipulator_msgs/srv/set_joint_position.hpp"


namespace oog
{
    class moveJoint : public rclcpp::Node{
    
    public:
        moveJoint(const std::string& name);
        int moveto(std::vector<double> q);

    private:
        rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr moveJointClient;
        std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition_Request> moveJointReq;
    };

} // namespace armService
