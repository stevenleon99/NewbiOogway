#include "rclcpp/rclcpp.hpp"
#include "open_manipulator_msgs/srv/set_kinematics_pose.hpp"

/*
Full stretch position from Home position
/// \max X: [0.134, 0.33]
/// \max Y: [-0.29, 0.29]
/// \max Z: [0.103, 0.32]
*/

namespace oog
{

class moveCart : public rclcpp::Node{

public:
    moveCart(const std::string& name);
    int moveto(std::vector<double> pose);

private:
    rclcpp::Client<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr moveCartClient;
    std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose_Request> moveCartReq;
};
    
} // namespace oog

