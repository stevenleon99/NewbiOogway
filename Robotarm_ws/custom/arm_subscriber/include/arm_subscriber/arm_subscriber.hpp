#ifndef ARM_SUBSCRIBER_H
#define ARM_SUBSCRIBER_H


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "open_manipulator_msgs/msg/kinematics_pose.hpp"
#include "open_manipulator_msgs/msg/open_manipulator_state.hpp"


namespace oog
{

    class Status_Subscriber : public rclcpp::Node
    {
    public:
        Status_Subscriber(const std::string& name);
        std::vector<double> joint = {0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<std::string> name = {"", "", "", "", ""};
        std::vector<double> velocity = {0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<double> effort =   {0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<double> position = {0.0, 0.0, 0.0};
        std::vector<double> orientation = {0.0, 0.0, 0.0, 0.0};
        std::string move;
        std::string enable;
        
    private:
        
        void callback_joint_topic(const sensor_msgs::msg::JointState & message) ;
        void callback_pose_topic(const open_manipulator_msgs::msg::KinematicsPose & message) ;
        void callback_state_topic(const open_manipulator_msgs::msg::OpenManipulatorState & message) ;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscription;
        rclcpp::Subscription<open_manipulator_msgs::msg::KinematicsPose>::SharedPtr poseSubscription;
        rclcpp::Subscription<open_manipulator_msgs::msg::OpenManipulatorState>::SharedPtr statusSubscription;
    };


} // namespace subscribe

#endif