/*
 * Discription:
 * Search Aruco in the workspace and try to pick-n-place after found Aruco
 * 
 * Aruco position topic: /aruco_single/position
 * 
 * Frame transformation
 * - End-effector frame: end_effector_link
 * -        Aruco frame: aruco_marker_frame
 * -         base frame: link1
 */

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "open_manipulator_msgs/srv/set_joint_position.hpp"
#include "open_manipulator_msgs/msg/kinematics_pose.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "open_manipulator_msgs/srv/set_kinematics_pose.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;


namespace oog
{
class search_aruco : public rclcpp::Node {

private:
    std::vector<double> search_position = {1.3775, -1.4419, 0.5369, 1.9512, 0.1};
    std::vector<double> search_range_joint1 = {1.3775, -1.8683};
    std::vector<double> prev_aruco_position = {0.0, 0.0 ,0.0};
    std::vector<double> cur_aruco_position = {0.0, 0.0 ,0.0};
    std::vector<std::vector<double>> search_joint_set;
    std::vector<std::vector<double>> search_aruco_set;

    double search_step = 0.10;
    u_int8_t max_step = 0;
    // flags
    bool isOpen = false;
    bool search_finish = false;
    bool ready_grasp = false;
    // aruco position
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr arucoPosition;
    // move joint service
    rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr moveJointClient;
    std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition_Request> moveJointReq;
    // move gripper tool service
    rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr moveToolClient;
    std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition_Request> moveToolReq;
    // create a timer 
    std::shared_ptr<rclcpp::TimerBase> timer_1;
    std::shared_ptr<rclcpp::TimerBase> timer_2;
    std::shared_ptr<rclcpp::TimerBase> timer_3;
    std::shared_ptr<rclcpp::TimerBase> timer_4;
    void _search();
    // create aruco position subscriber
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr arucoSubscription;
    void _callback_aruco_topic(const geometry_msgs::msg::Vector3Stamped & message);
    //
    void _to_ready_grasp();
    //
    void _grasp();
    //
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::TransformStamped t_link1_endeffector;
    geometry_msgs::msg::TransformStamped t_link1_aruco;
    void _lookuptransform();
    // move cart service
    rclcpp::Client<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr moveCartClient;
    std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose_Request> moveCartReq;


public:
    search_aruco(const std::string& name);
    int moveJointTo(std::vector<double> q);
    int moveToolTo(double q);
    int moveCartTo(std::vector<double> pose);
    double sumsquare(std::vector<double>& v1, std::vector<double>&v2);
    double sumsquare(std::vector<double>& v);
};
    
} // namespace oog

oog::search_aruco::search_aruco(const std::string& name):Node(name){
    std::cout << "start searching aruco..." << std::endl;

    // create client and request to move joint
    this->moveJointClient = this->create_client<open_manipulator_msgs::srv::SetJointPosition>("goal_joint_space_path");
    this->moveJointReq = std::make_shared<open_manipulator_msgs::srv::SetJointPosition_Request>();
    // move arm to ready position
    this->moveJointTo(this->search_position);
    // create client and request to move gripper tool
    this->moveToolClient = this->create_client<open_manipulator_msgs::srv::SetJointPosition>("goal_tool_control");
    this->moveToolReq = std::make_shared<open_manipulator_msgs::srv::SetJointPosition_Request>();
    // move gripper tool open
    this->moveToolTo(0.019);
    if (!this->isOpen) this->isOpen = true;
    // create timer wall
    this->timer_1 = create_wall_timer(4s, std::bind(&search_aruco::_search, this));
    this->timer_2 = create_wall_timer(5s, std::bind(&search_aruco::_to_ready_grasp, this));
    this->timer_3 = create_wall_timer(10s, std::bind(&search_aruco::_grasp, this));
    this->timer_4 = create_wall_timer(1s, std::bind(&search_aruco::_lookuptransform, this));
    // create aruco position subscriber
    this->arucoSubscription = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
                                        "aruco_single/position",
                                        10,
                                        std::bind(&search_aruco::_callback_aruco_topic, this, _1));
    
    this->tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // create cliet and request to move cart
    this->moveCartClient = this->create_client<open_manipulator_msgs::srv::SetKinematicsPose>("goal_task_space_path");
    this->moveCartReq = std::make_shared<open_manipulator_msgs::srv::SetKinematicsPose::Request>();

};
double oog::search_aruco::sumsquare(std::vector<double>& v){
    int n = v.size();
    double sum = 0;
    for (int i=0; i<n; i++){
        sum += pow(v[i], 2);
    }
    return sqrt(sum);
};


double oog::search_aruco::sumsquare(std::vector<double>& v1, std::vector<double>&v2){
    if (v1.size() != v2.size()){
        std::cout << "[search_aruco]: size does not match" << std::endl;
        return INT32_MAX;
    }
    int n = v1.size();
    double sum = 0;
    for (int i=0; i<n; i++){
        sum += pow((v1[i] - v2[i]), 2);
    }
    
    return sqrt(sum);
};

int oog::search_aruco::moveCartTo(std::vector<double> pose){
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
    this->moveCartReq->path_time = 5;

    try{
        auto future_result = this->moveCartClient->async_send_request(this->moveCartReq);
        // Wait 3s for the result
        rclcpp::sleep_for(5s);
        std::cout << "[search_aruco]: cart moved " << std::endl; 
    } catch (const std::exception& e) {
        std::cout << "[search_aruco]: standard error: " << e.what() << std::endl;
        return 0;
    }
    
    return 1;
};

void oog::search_aruco::_lookuptransform(){
    try {
        t_link1_aruco = tf_buffer_->lookupTransform("link1", "aruco_marker_frame", tf2::TimePointZero);
        // std::cout << t_link1_aruco.transform.translation.x << std::setw(10);
        // std::cout << t_link1_aruco.transform.translation.y << std::setw(10);
        // std::cout << t_link1_aruco.transform.translation.z << std::setw(10);
        // std::cout << std::endl;
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO( this->get_logger(), "Could not transform because %s", ex.what() ); 
    }
    
};


void oog::search_aruco::_grasp(){
    if (this->ready_grasp) {
        std::cout << "[search_aruco]: start grasping" << std::endl;
        double tx = this->t_link1_aruco.transform.translation.x;
        double ty = this->t_link1_aruco.transform.translation.y;
        double tz = this->t_link1_aruco.transform.translation.z;

        std::vector<double> target1 = {tx, ty, 0.10462, -0.254, 0.435, 0.436, 0.746};
        std::vector<double> target2 = {tx, ty, tz,      -0.254, 0.435, 0.436, 0.746};
        std::vector<double> target3 = {1.983437, -0.526155, 0.987884, 0.493942, 0.0};
        this->moveCartTo(target1);
        this->moveCartTo(target2);
        this->moveToolTo(0.0);
        rclcpp::sleep_for(2s);
        this->moveJointTo(target3);
        this->moveToolTo(0.019);

        this->timer_3->cancel();
        std::cout << "[search_aruco]: grasping ended" << std::endl;
    }
}

void oog::search_aruco::_to_ready_grasp(){
    if (this->search_finish){
        std::cout << "[search_aruco]: move to ready grasp" << std::endl;
        auto n = this->search_joint_set.size();
        this->moveJointTo(this->search_joint_set[n-2]);
        this->timer_2->cancel();
        this->ready_grasp = true;
    }
};


void oog::search_aruco::_callback_aruco_topic(const geometry_msgs::msg::Vector3Stamped & message){
    for (auto i=0; i<3; i++){this->prev_aruco_position[i] = this->cur_aruco_position[i];}
    this->cur_aruco_position[0] = message.vector.x;
    this->cur_aruco_position[1] = message.vector.y;
    this->cur_aruco_position[2] = message.vector.z;
    std::vector<double> in = {this->cur_aruco_position[0], this->cur_aruco_position[1]};

    if (this->sumsquare(in) <= 0.05){
        if (this->timer_1->is_steady()) {
            this->search_finish = true;
            this->timer_1->cancel();
        }

        std::cout << "[search_aruco]: search ended and target found" << std::endl;
        for (auto i=0; i<3; i++){
            std::cout << this->cur_aruco_position[i] << std::setw(10);
        }
        std::cout << std::endl;
    }

}


void oog::search_aruco::_search(){
    if (this->max_step >= 20) {
        std::cout << "[search_aruco]: search ended and no target found" << std::endl;
        this->timer_1->cancel();
    }
    if (!this->timer_1->is_canceled()){
        this->max_step += 1;
        // move joint around workspace
        this->search_position[0] -= this->search_step;
        this->moveJointTo(this->search_position);

        this->search_joint_set.push_back(this->search_position);
        if (cur_aruco_position[0] == 0.0 || this->sumsquare(this->prev_aruco_position, this->cur_aruco_position)<0.0001){
            this->search_aruco_set.push_back({-1,-1,-1}); // not exceeed +- 1.0m
        } else {
            this->search_aruco_set.push_back(this->cur_aruco_position);
        }
    }
}


int oog::search_aruco::moveJointTo(std::vector<double> q){
        if (q.size() != 5){
            RCLCPP_INFO(this->get_logger(), "[search_aruco]: size of input joint incorrect");
            return 0;
        }
        std::vector<std::string> jointname = {"joint1", "joint2", "joint3", "joint4", "gripper"};
        std::vector<double> jointposition = {q[0], q[1], q[2], q[3], q[4]};
        this->moveJointReq->joint_position.set__joint_name(jointname);
        this->moveJointReq->joint_position.set__position(jointposition);
        this->moveJointReq->path_time = 2.0; //second

        try{
            auto future_result = this->moveJointClient->async_send_request(this->moveJointReq);
            rclcpp::sleep_for(2s);
            std::cout << "[search_aruco]: joint moved " << std::endl; 
        } catch (const std::exception& e) {
            std::cout << "[search_aruco]: standard error: " << e.what() << std::endl;
            return 0;
        }

        return 1;
};

int oog::search_aruco::moveToolTo(double q){
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
            // Wait for 1s
            rclcpp::sleep_for(1s);
            std::cout << "[search_aruco]: tool moved " << std::endl;
        } catch (const std::exception& e) {
            std::cout << "[search_aruco]: standard error: " << e.what() << std::endl;
            return 0;
        }
        
        return 1;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto search_aruco_node =  std::make_shared<oog::search_aruco>("search_aruco");
    rclcpp::spin(search_aruco_node);
    rclcpp::shutdown();
    
    return 0;
}
