#include "rclcpp/rclcpp.hpp"
#include "open_manipulator_msgs/srv/set_joint_position.hpp"


namespace oog
{
    class moveTool : public rclcpp::Node{
    
    public:
        moveTool(const std::string& name);
        int moveto(double q);

    private:
        rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr moveToolClient;
        std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition_Request> moveToolReq;
    };

} // namespace armService
