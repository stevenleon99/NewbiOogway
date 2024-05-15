#include "arm_service/arm_movejoint.hpp"
#include "arm_service/arm_movetool.hpp"
#include "arm_service/arm_movecart.hpp"

using namespace std::chrono_literals;

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node_movejoint = std::make_shared<oog::moveJoint>("movejointnode_pnp");
    auto node_movetool = std::make_shared<oog::moveTool>("movetoolnode_pnp");
    std::vector<double> pick_ready = {1.392855, -0.230097, 0.018408, 1.765612, 0};
    double open = 0.024;
    std::vector<double> pick = {1.326893, -0.282252, 0.457126, 1.409728, 0};
    double close = -0.01;
    std::vector<double> intermediate = {0.070563, -0.358952, 0.082835, 1.141282, 0};
    std::vector<double> place_ready = {-0.460194, -0.078233, -0.069029, 1.641359, 0};
    std::vector<double> place = {-0.403437, 0.053689, -0.027612, 1.418932, 0};
    

    if (rclcpp::ok()){
        
        node_movejoint->moveto(pick_ready);
        rclcpp::sleep_for(2s);
        node_movetool->moveto(open);
        rclcpp::sleep_for(2s);
        node_movejoint->moveto(pick);
        rclcpp::sleep_for(2s);
        node_movetool->moveto(close);
        rclcpp::sleep_for(2s);
        node_movejoint->moveto(intermediate);
        rclcpp::sleep_for(2s);
        node_movejoint->moveto(place_ready);
        rclcpp::sleep_for(2s);
        node_movejoint->moveto(place);
        rclcpp::sleep_for(2s);
        node_movetool->moveto(open);
        rclcpp::sleep_for(2s);
        node_movejoint->moveto(intermediate);
        rclcpp::sleep_for(2s);
        std::cout << "pnp done" << std::endl;
        }


    rclcpp::shutdown();
    
    return 0;
}
