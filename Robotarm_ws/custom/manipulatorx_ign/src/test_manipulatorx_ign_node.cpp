#include "manipulatorx_ign/test_manipulatorx_ign.hpp"


int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin( std::make_shared<oog::test_manipulatorx_ign>("mx") );
    rclcpp::shutdown();

    return 0;
}
