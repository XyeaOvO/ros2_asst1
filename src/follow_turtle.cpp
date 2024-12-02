#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

class node: public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target,catcher;
    bool reach_flag;

    // PID控制器变量
    double Kp_linear, Ki_linear, Kd_linear;
    double Kp_angular, Ki_angular, Kd_angular;

    double prev_error_linear, prev_error_angular;
    double integral_linear, integral_angular;

    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped tf;     
        try
        {
            tf = tf_buffer_->lookupTransform(catcher, target, tf2::TimePointZero);
        } 
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s", catcher.c_str(), target.c_str(), ex.what());
            return;
        }

        auto t = tf.transform;
        auto message = geometry_msgs::msg::Twist();

        // 计算线性误差和角度误差
        double error_linear = hypot(t.translation.x, t.translation.y); 
        double error_angular = atan2(t.translation.y, t.translation.x);

        // PID控制计算
        double control_linear = Kp_linear * error_linear + Ki_linear * integral_linear + Kd_linear * (error_linear - prev_error_linear);
        double control_angular = Kp_angular * error_angular + Ki_angular * integral_angular + Kd_angular * (error_angular - prev_error_angular);

        // 更新积分项和前一个误差
        integral_linear += error_linear;
        integral_angular += error_angular;
        prev_error_linear = error_linear;
        prev_error_angular = error_angular;

        // 如果误差小于阈值，停止
        if (error_linear < 0.1)
        {
            message.linear.x = 0.0;
            message.angular.z = 0.0;
            publisher_->publish(message);
            if (!reach_flag) RCLCPP_INFO(this->get_logger(), "Reached target");
            reach_flag = true;
            return;
        }
        else
        {
            reach_flag = false;
        }

        // 设定限额
        const double linear_min = -100.0;
        const double linear_max = 100.0;
        const double angular_min = -1000.0;
        const double angular_max = 1000.0;

        // 限制线速度
        if (control_linear < linear_min) {
            control_linear = linear_min;
        } else if (control_linear > linear_max) {
            control_linear = linear_max;
        }

        // 限制角速度
        if (control_angular < angular_min) {
            control_angular = angular_min;
        } else if (control_angular > angular_max) {
            control_angular = angular_max;
        }

        // 发布控制命令
        message.linear.x = control_linear;
        message.angular.z = control_angular;
        RCLCPP_INFO(this->get_logger(), "Publishing: linear.x: '%f', angular.z: '%f'", message.linear.x, message.angular.z);
        publisher_->publish(message);
    }

public: 
    node(std::string target, std::string catcher): Node("follower"), target(target), catcher(catcher)
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&node::timer_callback, this));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(catcher+"/cmd_vel", 10);

        // 初始化PID控制参数
        Kp_linear = 3; Ki_linear = 0.005; Kd_linear = 0.2;
        Kp_angular = 15; Ki_angular = 0.0; Kd_angular = 0;

        prev_error_linear = 0.0;
        prev_error_angular = 0.0;
        integral_linear = 0.0;
        integral_angular = 0.0;

        RCLCPP_INFO(this->get_logger(), "Hello, world");   
    }

    ~node()
    {
        RCLCPP_INFO(this->get_logger(), "Goodbye, world");
    }
};

class ROS_EVENT_LOOP
{
public:
    ROS_EVENT_LOOP(int argc, char *argv[], std::string target,std::string catcher)
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<node>(target,catcher));
    }
    ~ROS_EVENT_LOOP()
    {
        rclcpp::shutdown();
    }
};

int main(int argc, char *argv[])
{
    if (argc != 3 )
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: follow target catcher");
        return 1;
    }
    ROS_EVENT_LOOP(argc, argv, argv[1], argv[2]);
    return 0;
}
