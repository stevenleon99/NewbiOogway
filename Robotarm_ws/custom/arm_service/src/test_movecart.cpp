#include "arm_service/arm_movecart.hpp"
#include "arm_service/arm_movejoint.hpp"

using namespace std::chrono_literals;


int main(int argc, char const *argv[])
{
    std::vector<double> target_pose = {0.136, -0.000, 0.234, 0.000, 0.016, -0.001, 1.000};
    double delta = 0.01;

    // move to ready position
    std::vector<double> ready = {0.000000, -1.052311, 0.357418, 0.701029, 0.0};

    rclcpp::init(argc, argv);

    // create move joint
    auto node_movejoint = std::make_shared<oog::moveJoint>("movejoint");
    // create move cart
    auto node_movecart = std::make_shared<oog::moveCart>("movecart");

    std::cout << "select predefine position: w:+x, s:-x, a:+y, d:-y, r:ready, q:quit" <<std::endl;
    char select = ' ';
    while (1) {
        std::cin >> select;
        if (select == 'q') break;
        switch (select)
        {
        case 'w':
            /* code */
            if (rclcpp::ok()){
                target_pose[0] += delta;
                auto res = node_movecart->moveto(target_pose);
                std::cout << "move joint res: " << res << std::endl;
            }
            break;
        
        case 's':
            /* code */
            if (rclcpp::ok()){
                target_pose[0] -= delta;
                auto res = node_movecart->moveto(target_pose);
                std::cout << "move joint res: " << res << std::endl;
            }
            break;

        case 'a':
            /* code */
            if (rclcpp::ok()){
                target_pose[1] += delta;
                auto res = node_movecart->moveto(target_pose);
                std::cout << "move joint res: " << res << std::endl;
            }
            break;
        
        case 'd':
            /* code */
            if (rclcpp::ok()){
                target_pose[1] -= delta;
                auto res = node_movecart->moveto(target_pose);
                std::cout << "move joint res: " << res << std::endl;
            }
            break;
        
        case 'r':
            /* code */
            if (rclcpp::ok()){
                auto res = node_movejoint->moveto(ready);
                std::cout << "move joint res: " << res << std::endl;
            }
            break;
        
        default:
            std::cout << "please select w or s or a or d or r" << std::endl;
            break;
        }
    }

    
    rclcpp::shutdown();

    return 0;
}
