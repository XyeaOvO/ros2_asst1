#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float64.hpp"  // 添加用于发布误差值
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <vector>
#include <memory>

class FormationFollowerNode: public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    // 添加误差发布器
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_linear_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_angular_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target, catcher;
    bool reach_flag;
    tf2::Transform initial_relative_transform_;
    bool initial_transform_recorded_ = false;

    // PID控制器变量
    double Kp_linear, Ki_linear, Kd_linear;
    double Kp_angular, Ki_angular, Kd_angular;
    double prev_error_linear, prev_error_angular;
    double integral_linear, integral_angular;

    void record_initial_transform()
    {
        geometry_msgs::msg::TransformStamped tf;
        try
        {
            tf = tf_buffer_->lookupTransform(catcher, target, tf2::TimePointZero);
            tf2::Transform current_transform;
            tf2::fromMsg(tf.transform, current_transform);
            initial_relative_transform_ = current_transform;
            initial_transform_recorded_ = true;
            RCLCPP_INFO(this->get_logger(), "Recorded initial transform between %s and %s",
                       catcher.c_str(), target.c_str());
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not record initial transform: %s", ex.what());
        }
    }

    void publish_error(double error_linear, double error_angular) {
        auto linear_msg = std_msgs::msg::Float64();
        auto angular_msg = std_msgs::msg::Float64();
        
        linear_msg.data = error_linear;
        angular_msg.data = error_angular;
        
        error_linear_publisher_->publish(linear_msg);
        error_angular_publisher_->publish(angular_msg);
    }

    void timer_callback()
    {
        if (!initial_transform_recorded_)
        {
            record_initial_transform();
            return;
        }

        geometry_msgs::msg::TransformStamped current_tf;
        try
        {
            current_tf = tf_buffer_->lookupTransform(catcher, target, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s",
                        catcher.c_str(), target.c_str(), ex.what());
            return;
        }

        tf2::Transform current_transform;
        tf2::fromMsg(current_tf.transform, current_transform);
        
        tf2::Transform error_transform = current_transform * initial_relative_transform_.inverse();
   
        double error_linear = std::hypot(error_transform.getOrigin().x(),
                                       error_transform.getOrigin().y());
        double error_angular = std::atan2(error_transform.getOrigin().y(),
                                        error_transform.getOrigin().x());

        // 发布误差值用于可视化
        publish_error(error_linear, error_angular);

        double control_linear = Kp_linear * error_linear +
                              Ki_linear * integral_linear +
                              Kd_linear * (error_linear - prev_error_linear);
        double control_angular = Kp_angular * error_angular +
                               Ki_angular * integral_angular +
                               Kd_angular * (error_angular - prev_error_angular);

        integral_linear += error_linear;
        integral_angular += error_angular;
        prev_error_linear = error_linear;
        prev_error_angular = error_angular;

        if (error_linear < 0.1)
        {
            auto message = geometry_msgs::msg::Twist();
            message.linear.x = 0.0;
            message.angular.z = 0.0;
            integral_angular = 0.0;
            integral_linear -= 5.0;
            publisher_->publish(message);
            if (!reach_flag) {
                RCLCPP_INFO(this->get_logger(), "%s reached formation position relative to %s",
                           catcher.c_str(), target.c_str());
            }
            reach_flag = true;
            return;
        }
        else
        {
            reach_flag = false;
        }

        control_linear = std::clamp(control_linear, -30.0, 30.0);
        control_angular = std::clamp(control_angular, -30.0, 30.0);

        auto message = geometry_msgs::msg::Twist();
        message.linear.x = control_linear;
        message.angular.z = control_angular;
        publisher_->publish(message);
    }

public:
    FormationFollowerNode(const std::string& target, const std::string& catcher)
        : Node("formation_follower_" + catcher), target(target), catcher(catcher)
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&FormationFollowerNode::timer_callback, this));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            catcher + "/cmd_vel", 10);
            
        // 创建误差发布器
        error_linear_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            catcher + "/error_linear", 10);
        error_angular_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            catcher + "/error_angular", 10);

        Kp_linear = 5.0; Ki_linear = 0.002; Kd_linear = 0.5;
        Kp_angular = 5.0; Ki_angular = 0.00; Kd_angular = 0.0;    

        prev_error_linear = 0.0;
        prev_error_angular = 0.0;
        integral_linear = 0.0;
        integral_angular = 0.0;

        RCLCPP_INFO(this->get_logger(), "Started formation follower for %s following %s",
                    catcher.c_str(), target.c_str());
    }

    ~FormationFollowerNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down formation follower for %s", catcher.c_str());
    }
};

class MultiFormationFollowerManager
{
private:
    std::vector<std::shared_ptr<FormationFollowerNode>> nodes;
    rclcpp::executors::MultiThreadedExecutor executor;

public:
    void addFollower(const std::string& target, const std::string& follower) {
        nodes.push_back(std::make_shared<FormationFollowerNode>(target, follower));
        executor.add_node(nodes.back());
    }

    void spin() {
        executor.spin();
    }
};

int main(int argc, char *argv[])
{
    if (argc < 3) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                    "Usage: follow target follower1 [follower2 follower3 ...]");
        return 1;
    }

    rclcpp::init(argc, argv);

    MultiFormationFollowerManager manager;

    const std::string target = argv[1];

    for (int i = 2; i < argc; i++) {
        manager.addFollower(target, argv[i]);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Added formation follower %s to follow %s", argv[i], target.c_str());
    }

    manager.spin();

    rclcpp::shutdown();
    return 0;
}