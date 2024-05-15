#include "arm_service/arm_movejoint.hpp"

using namespace std::chrono_literals;

namespace oog
{
    
    moveJoint::moveJoint(const std::string& name): Node(name){
        this->moveJointClient = this->create_client<open_manipulator_msgs::srv::SetJointPosition>("goal_joint_space_path");
        this->moveJointReq = std::make_shared<open_manipulator_msgs::srv::SetJointPosition_Request>();

    };

    int moveJoint::moveto(std::vector<double> q){
        if (q.size() != 5){
            RCLCPP_INFO(this->get_logger(), "[arm_moveJoint]: size of input joint incorrect");
            return 0;
        }
        std::vector<std::string> jointname = {"joint1", "joint2", "joint3", "joint4", "gripper"};
        std::vector<double> jointposition = {q[0], q[1], q[2], q[3], q[4]};
        this->moveJointReq->joint_position.set__joint_name(jointname);
        this->moveJointReq->joint_position.set__position(jointposition);
        this->moveJointReq->path_time = 2.0; //second

        try{
            auto future_result = this->moveJointClient->async_send_request(this->moveJointReq);
            // Wait indefinitely for the result
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) !=
                rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "[arm_moveJoint] Send goal call failed");
                return 0;
            }
            std::cout << "[arm_moveJoint]: tool moved " << std::endl; 
        } catch (const std::exception& e) {
            std::cout << "[arm_movejoint]: standard error: " << e.what() << std::endl;
            return 0;
        }
        
        return 1;
    };
}
// namespace armJointService