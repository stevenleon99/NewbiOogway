#include "arm_subscriber/arm_subscriber.hpp"

using std::placeholders::_1;

namespace oog
{
    Status_Subscriber::Status_Subscriber(const std::string& name):Node(name){
        this->jointSubscription = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states",
            10,
            std::bind(&Status_Subscriber::callback_joint_topic, this, _1)
            );
        this->poseSubscription = this->create_subscription<open_manipulator_msgs::msg::KinematicsPose>(
            "kinematics_pose",
            10,
            std::bind(&Status_Subscriber::callback_pose_topic, this, _1)
            );
        this->statusSubscription = this->create_subscription<open_manipulator_msgs::msg::OpenManipulatorState>(
            "states",
            10,
            std::bind(&Status_Subscriber::callback_state_topic, this, _1)
            );
    };
    
    void Status_Subscriber::callback_joint_topic(const sensor_msgs::msg::JointState & message) {
        try{
            for (int i=0; i<5; i++){
                this->joint[i] = message.position[i];
                this->name[i] = message.name[i];
                this->velocity[i] = message.velocity[i];
                this->effort[i] = message.effort[i];
            }
        } catch (const std::exception& e) {
            std::cout << "[arm_subscriber]: standard error: " << e.what() << std::endl;
        }
        
        std::cout << "receive joint: " << this->joint.size() << std::endl;
        // std::cout << "receive joint: " << "1: " << this->joint[0] << std::endl;
        // std::cout << "receive joint: " << "2: " << this->joint[1] << std::endl;
        // std::cout << "receive joint: " << "3: " << this->joint[2] << std::endl;
        // std::cout << "receive joint: " << "4: " << this->joint[3] << std::endl;
        // std::cout << "receive joint: " << "5: " << this->joint[4] << std::endl;
    }

    void Status_Subscriber::callback_pose_topic(const open_manipulator_msgs::msg::KinematicsPose & message) {
        this->position[0] = message.pose.position.x;
        this->position[1] = message.pose.position.y;
        this->position[2] = message.pose.position.z;
        
        this->orientation[0] = message.pose.orientation.x;
        this->orientation[1] = message.pose.orientation.y;
        this->orientation[2] = message.pose.orientation.z;
        this->orientation[3] = message.pose.orientation.w;
        std::cout << "receive position x: " << this->position[0] << std::endl;
        std::cout << "receive position y: " << this->position[1] << std::endl;
        std::cout << "receive position z: " << this->position[2] << std::endl;
        std::cout << "receive orientation x: " << this->orientation[0] << std::endl;
        std::cout << "receive orientation y: " << this->orientation[1] << std::endl;
        std::cout << "receive orientation z: " << this->orientation[2] << std::endl;
        std::cout << "receive orientation w: " << this->orientation[3] << std::endl;

    }
    void Status_Subscriber::callback_state_topic(const open_manipulator_msgs::msg::OpenManipulatorState & message) {
        this->move = message.open_manipulator_moving_state;
        this->enable = message.open_manipulator_actuator_state;
        std::cout << "receive move: " << this->move << std::endl;

    }

} // namespace subscribe