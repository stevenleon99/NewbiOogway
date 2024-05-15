#include "arm_service/arm_movetool.hpp"


namespace oog
{

    moveTool::moveTool(const std::string& name) : Node(name){
        this->moveToolClient = this->create_client<open_manipulator_msgs::srv::SetJointPosition>("goal_tool_control");
        this->moveToolReq = std::make_shared<open_manipulator_msgs::srv::SetJointPosition_Request>();

    };

    int moveTool::moveto(double q){
        // if (q.size() != 5){
        //     RCLCPP_INFO(this->get_logger(), "[arm_moveTool]: size of input joint incorrect");
        //     return 0;
        // }
        std::vector<std::string> jointname = {"gripper"};
        std::vector<double> jointposition = {q};
        this->moveToolReq->joint_position.set__joint_name(jointname);
        this->moveToolReq->joint_position.set__position(jointposition);
        this->moveToolReq->path_time = 1.0; //second
        
        try{
            auto future_result = this->moveToolClient->async_send_request(this->moveToolReq);
            // Wait indefinitely for the result
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) !=
                rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "[arm_moveJoint] Send goal call failed");
                return 0;
            }
            std::cout << "[arm_moveTool]: tool moved " << std::endl;
        } catch (const std::exception& e) {
            std::cout << "[arm_moveTool]: standard error: " << e.what() << std::endl;
            return 0;
        }
        
        return 1;
    };

    
} // namespace armToolService

