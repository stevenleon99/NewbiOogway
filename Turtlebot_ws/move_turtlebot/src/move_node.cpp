#include <move_turtlebot/move.hpp>

int main( int argc, char** argv ){

  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<Move>("move_d"));
  rclcpp::shutdown();

  return 0;
}