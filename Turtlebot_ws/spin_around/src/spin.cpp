#include "spin_around/spin.hpp"


Spin::Spin(const std::string& name) : Node(name) {
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
        "/aruco_poses", 10, std::bind(&Spin::ar_cb, this, std::placeholders::_1)
    );

    this->tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer);

    this->spin_timer = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&Spin::spin_cb, this)
    );

    this->cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}


Eigen::Matrix4d Spin::quat2matrix(const Eigen::Quaterniond& quat, const Eigen::Vector3d& pos) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3, 3>(0, 0) = quat.toRotationMatrix();
    matrix.block<3, 1>(0, 3) = pos;
    return matrix;
}

void Spin::ar_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    std::cout << "spin_ar_cb" << std::endl;
    try {
        // -------------- // "px100/base_link" maybe change
        auto trans = this->tf_buffer->lookupTransform("base_link", "oakd_rgb_camera_frame", rclcpp::Time(0));

        Eigen::Quaterniond orientation(trans.transform.rotation.w,
                                       trans.transform.rotation.x,
                                       trans.transform.rotation.y,
                                       trans.transform.rotation.z);

        Eigen::Vector3d position(trans.transform.translation.x,
                                 trans.transform.translation.y,
                                 trans.transform.translation.z);

        this->camera2base_Matrix = this->quat2matrix(orientation, position);
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

void Spin::spin_cb() {
    RCLCPP_INFO(this->get_logger(), "spin_cb");
    std::stringstream ss;
    ss << this->marker2base_Matrix;
    RCLCPP_INFO(this->get_logger(), "Matrix: \n%s", ss.str().c_str());


    if (this->marker2base_Matrix.isApprox(Eigen::Matrix4d::Identity())) {
        RCLCPP_INFO(this->get_logger(), "No marker detected");
        this->move_cmd.angular.z = 0.3;
        this->cmd_pub->publish(this->move_cmd);
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Marker detected");
        auto relative = Eigen::Vector2d(this->marker2base_Matrix(0, 3), this->marker2base_Matrix(1, 3));
        auto angle = std::atan2(relative(1), relative(0));
        RCLCPP_INFO(this->get_logger(), "position: [%f, %f], angle: %f", relative.x(), relative.y(), angle);

        double thred = (5.0) * M_PI / 180.0;
        
        if (std::abs(angle) < thred) {
            this->move_cmd.angular.z = 0.0;
            this->cmd_pub->publish(this->move_cmd);
            this->spin_timer->cancel();
            return;
        }
        else {
            auto z = angle / 3.0;
            this->move_cmd.angular.z = z;
            RCLCPP_INFO(this->get_logger(), "z: %f", z);
            this->cmd_pub->publish(this->move_cmd);
        }
    }

}



