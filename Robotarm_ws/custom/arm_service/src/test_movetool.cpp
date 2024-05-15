#include "arm_service/arm_movetool.hpp"

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node_movetool = std::make_shared<oog::moveTool>("moveJointNode");
    // std::vector<double> base = {0.0, 0.0, 0.0, 0.0, 0.0};
    double base;

    std::cout << "select predefine position: 1:open, 2:close, 3:quit " <<std::endl;
    int select = -1;
    while (1) {
        std::cin >> select;
        if (select == 3) break;
        switch (select)
        {
        case 1:
            /* code */
            if (rclcpp::ok()){
                base = 0.023;
                auto res = node_movetool->moveto(base);
                std::cout << "move tool res: " << res << std::endl;
            }
            break;
        
        case 2:
            /* code */
            if (rclcpp::ok()){
                base = -0.01;
                auto res = node_movetool->moveto(base);
                std::cout << "move tool res: " << res << std::endl;
            }
            break;
        
        default:
            std::cout << "please select 1 or 2 " << std::endl;
            break;
        }
    }

    
    rclcpp::shutdown();

    return 0;
}