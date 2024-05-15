#include "manipulatorx_ign/test_manipulatorx_ign.hpp"

typedef oog::test_manipulatorx_ign manipulator;
typedef sensor_msgs::msg::JointState JointState;

using namespace std::chrono_literals;

namespace oog
{

manipulator::test_manipulatorx_ign(const std::string& name): Node(name){
    
    this->joint_pub = create_publisher<JointState>("joint_states", 10);
    //create timer
    this->timer = create_wall_timer(1000ms, std::bind(&manipulator::_movejoint_callback, this));
    this->velocity_pub = create_publisher<std_msgs::msg::Float64MultiArray>("joint_velocities/commands", 1);
};

void manipulator::_movejoint_callback(){
    if (this->cur_vel < this->target_vel){

        this->cur_vel += this->stp_vel;

        //publish the velocity
        std::vector<double> ret(this->cur_joint);
        ret[0] = this->cur_vel;
        ret[4] = 0.001;
        ret[5] = 0.001;
        std::cout << "[test_manipulatorx_ign]: current velocity is " << ret[0] << std::endl;
        std_msgs::msg::Float64MultiArray v;
        std_msgs::msg::MultiArrayDimension dim;
        dim.label = std::string("velocity");
        dim.size = ret.size();
        dim.stride = 1;
        v.layout.dim.push_back(dim);
        v.data = ret;
        this->velocity_pub->publish(v);
    }
    else {
        std::vector<double> ret(this->cur_joint);
        std_msgs::msg::Float64MultiArray v;
        std_msgs::msg::MultiArrayDimension dim;
        dim.label = std::string("velocity");
        dim.size = ret.size();
        dim.stride = 1;
        v.layout.dim.push_back(dim);
        v.data = ret;
        this->velocity_pub->publish(v);
        
        RCLCPP_INFO(get_logger(), "reached target velocity");
    }

};


} // namespace oog
