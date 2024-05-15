#include "arm_service/arm_movecart.hpp"

namespace oog
{

moveCart::moveCart(const std::string& name): Node(name){
    this->moveCartClient = this->create_client<open_manipulator_msgs::srv::SetKinematicsPose>("goal_task_space_path");
    this->moveCartReq = std::make_shared<open_manipulator_msgs::srv::SetKinematicsPose::Request>();
};

/**
    \ param pose: [x, y, z, w, q0, q1, q2]
*/
int moveCart::moveto(std::vector<double> pose){
    if (pose.size() != 7){
            RCLCPP_INFO(this->get_logger(), "[arm_moveCart]: size of input pose incorrect");
            return 0;
        }
    
    // Set the position (x, y, z)
    this->moveCartReq->end_effector_name = "gripper";
    this->moveCartReq->kinematics_pose.pose.position.x = pose[0];
    this->moveCartReq->kinematics_pose.pose.position.y = pose[1];
    this->moveCartReq->kinematics_pose.pose.position.z = pose[2];
    this->moveCartReq->kinematics_pose.pose.orientation.x = pose[3];
    this->moveCartReq->kinematics_pose.pose.orientation.y = pose[4];
    this->moveCartReq->kinematics_pose.pose.orientation.z = pose[5];
    this->moveCartReq->kinematics_pose.pose.orientation.w = pose[6];
    // this->moveCartReq->kinematics_pose.max_accelerations_scaling_factor = 0.0;
    // this->moveCartReq->kinematics_pose.max_velocity_scaling_factor = 0.0;
    // this->moveCartReq->kinematics_pose.tolerance = 0.0;
    this->moveCartReq->path_time = 0.5; //second

    try{
        auto future_result = this->moveCartClient->async_send_request(this->moveCartReq);
        // Wait indefinitely for the result
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "[arm_moveCart] Send goal call failed");
            return 0;
        }
        std::cout << "[arm_moveCart]: tool moved " << std::endl; 
    } catch (const std::exception& e) {
        std::cout << "[arm_moveCart]: standard error: " << e.what() << std::endl;
        return 0;
    }
    
    return 1;
};

} // namespace oog




