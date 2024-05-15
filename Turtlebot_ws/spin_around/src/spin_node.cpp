#include <spin_around/spin.hpp>

int main( int argc, char** argv ){

  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<Spin>("spin"));
  rclcpp::shutdown();

  return 0;
}