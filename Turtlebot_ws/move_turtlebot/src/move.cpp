#include "move_turtlebot/move.hpp"

Move::Move(const std::string& name) : Node(name) {
    this->marker2camera_Matrix = Eigen::Matrix4d::Identity();
    this->camera2base_Matrix = Eigen::Matrix4d::Identity();
    this->marker2base_Matrix = Eigen::Matrix4d::Identity();
    this->rot_matrix1 << 1., 0., 0., 0.,
                        0., 1., 0., 0.,
                        0., 0., 1., 0.,
                        0., 0., 0., 1.;
    this->rot_matrix2 << 1., 0., 0., 0.,
                        0., 1., 0., 0.,
                        0., 0., 1., 0.,
                        0., 0., 0., 1.;
    
    this->marker_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/aruco_poses", 10, std::bind(&Move::ar_cb, this, std::placeholders::_1)
    );

    this->tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer);

    this->move_timer = this->create_wall_timer(
        std::chrono::milliseconds(1000), std::bind(&Move::move_cb, this)
    );

    this->cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

}

Eigen::Matrix4d Move::quat2matrix(const Eigen::Quaterniond& quat, const Eigen::Vector3d& pos) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3, 3>(0, 0) = quat.toRotationMatrix();
    matrix.block<3, 1>(0, 3) = pos;
    return matrix;
}

void Move::ar_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    // std::cout << "ar_cb" << std::endl;
    try {
        // ------------------------------------------------------------------------------------ // "base_link" maybe change
        auto trans = this->tf_buffer->lookupTransform("base_link", "oakd_rgb_camera_optical_frame", rclcpp::Time(0));

        Eigen::Quaterniond orientation(trans.transform.rotation.w,
                                       trans.transform.rotation.x,
                                       trans.transform.rotation.y,
                                       trans.transform.rotation.z);

        Eigen::Vector3d position(trans.transform.translation.x,
                                 trans.transform.translation.y,
                                 trans.transform.translation.z);

        this->camera2base_Matrix = quat2matrix(orientation, position);
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        return;
    }

    auto pose = msg->poses[0];
    Eigen::Vector3d pos(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    this->marker2camera_Matrix = this->quat2matrix(quat, pos);
    this->marker2base_Matrix = this->camera2base_Matrix * this->rot_matrix2 * this->marker2camera_Matrix * this->rot_matrix1;
}

void Move::move_cb() {
    RCLCPP_INFO(this->get_logger(), "move_cb");
    std::stringstream ss;
    ss << this->marker2base_Matrix;
    RCLCPP_INFO(this->get_logger(), "m2bMatrix: \n%s", ss.str().c_str());
    std::stringstream ss2;
    ss2 << this->marker2camera_Matrix;
    RCLCPP_INFO(this->get_logger(), "m2cMatrix: \n%s", ss2.str().c_str());
    std::stringstream ss3;
    ss3 << this->camera2base_Matrix;
    RCLCPP_INFO(this->get_logger(), "c2bMatrix: \n%s", ss3.str().c_str());


    if (this->marker2base_Matrix.isApprox(Eigen::Matrix4d::Identity())) {
        RCLCPP_INFO(this->get_logger(), "No marker detected");
        this->move_cmd.linear.x = 0.0;
        this->move_cmd.linear.y = 0.0;
        this->cmd_pub->publish(this->move_cmd);
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Marker detected");
        auto relative = Eigen::Vector2d(this->marker2base_Matrix(0, 3), this->marker2base_Matrix(1, 3));
        double distance = relative.x()*relative.x() + relative.y()*relative.y();
        // RCLCPP_INFO(this->get_logger(), "position: [%f, %f]", relative.x(), relative.y());
        RCLCPP_INFO(this->get_logger(), "distance to tag: %f", distance);

        double safe_distance = 0.1;  // -------------- // maybe need change
        if (distance < safe_distance) {
            this->move_cmd.linear.x = 0.0;
            this->cmd_pub->publish(this->move_cmd);
            this->move_timer->cancel();
            return;
        }
        else {
            this->move_cmd.linear.x = distance - safe_distance;
            this->cmd_pub->publish(this->move_cmd);
        }
    }

}



