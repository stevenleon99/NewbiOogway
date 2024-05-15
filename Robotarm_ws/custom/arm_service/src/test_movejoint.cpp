#include "arm_service/arm_movejoint.hpp"

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node_movejoint = std::make_shared<oog::moveJoint>("moveJointNode");
    std::vector<double> ready = {0.000000, -1.052311, 0.357418, 0.701029, 0.0};
    std::vector<double> home  = {-0.001534, 0.009204, 0.016874, 0.004602, 0.0};
    std::vector<double> storage = {-0.440253, -0.036816, 0.222427, 1.136680, 0.0};
    std::cout << "select predefine position: 1:ready, 2:home, 3:storage, 4:quit" <<std::endl;
    int select = -1;
    while (1) {
        std::cin >> select;
        if (select == 4) break;
        switch (select)
        {
        case 1:
            /* code */
            if (rclcpp::ok()){
            auto res = node_movejoint->moveto(ready);
            std::cout << "move joint res: " << res << std::endl;
            }
            break;
        
        case 2:
            /* code */
            if (rclcpp::ok()){
            auto res = node_movejoint->moveto(home);
            std::cout << "move joint res: " << res << std::endl;
            }
            break;

        case 3:
            /* code */
            if (rclcpp::ok()){
            auto res = node_movejoint->moveto(storage);
            std::cout << "move joint res: " << res << std::endl;
            }
            break;
        
        default:
            std::cout << "please select 1 or 2 or 3" << std::endl;
            break;
        }
    }

    
    rclcpp::shutdown();

    return 0;
}
