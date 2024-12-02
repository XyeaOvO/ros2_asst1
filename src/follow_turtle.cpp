#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include <vector>
#include <memory>

class FollowerNode: public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target, catcher;
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
            RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s", 
                        catcher.c_str(), target.c_str(), ex.what());
            return;
        }

        auto t = tf.transform;
        auto message = geometry_msgs::msg::Twist();

        // 计算线性误差和角度误差
        double error_linear = hypot(t.translation.x, t.translation.y); 
        double error_angular = atan2(t.translation.y, t.translation.x);

        // PID控制计算
        double control_linear = Kp_linear * error_linear + 
                              Ki_linear * integral_linear + 
                              Kd_linear * (error_linear - prev_error_linear);
        double control_angular = Kp_angular * error_angular + 
                               Ki_angular * integral_angular + 
                               Kd_angular * (error_angular - prev_error_angular);

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
            integral_angular = 0.0;
            integral_linear -= 5.0;
            publisher_->publish(message);
            if (!reach_flag) {
                RCLCPP_INFO(this->get_logger(), "%s reached target %s", 
                           catcher.c_str(), target.c_str());
            }
            reach_flag = true;
            return;
        }
        else
        {
            reach_flag = false;
        }

        // 设定限额并限制速度
        control_linear = std::clamp(control_linear, -30.0, 30.0);
        control_angular = std::clamp(control_angular, -30.0, 30.0);

        // 发布控制命令
        message.linear.x = control_linear;
        message.angular.z = control_angular;
        publisher_->publish(message);
    }

public: 
    FollowerNode(const std::string& target, const std::string& catcher)
        : Node("follower_" + catcher), target(target), catcher(catcher)
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&FollowerNode::timer_callback, this));
            
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            catcher + "/cmd_vel", 10);

        // 初始化PID控制参数
        // Kp_linear = 5.0; Ki_linear = 0.1; Kd_linear = 0.2;
        // Kp_angular = 10.0; Ki_angular = 0; Kd_angular = 0.0;

        Kp_linear = 2.0; Ki_linear = 0.05; Kd_linear = 0.1;
        Kp_angular = 10.0; Ki_angular = 0.05; Kd_angular = 0.1;    

        prev_error_linear = 0.0;
        prev_error_angular = 0.0;
        integral_linear = 0.0;
        integral_angular = 0.0;

        RCLCPP_INFO(this->get_logger(), "Started follower for %s following %s", 
                    catcher.c_str(), target.c_str());   
    }

    ~FollowerNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down follower for %s", catcher.c_str());
    }
};

class MultiFollowerManager 
{
private:
    std::vector<std::shared_ptr<FollowerNode>> nodes;
    rclcpp::executors::MultiThreadedExecutor executor;

public:
    void addFollower(const std::string& target, const std::string& follower) {
        nodes.push_back(std::make_shared<FollowerNode>(target, follower));
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
    
    MultiFollowerManager manager;
    
    // 获取目标名称
    const std::string target = argv[1];
    
    // 添加所有跟随者
    for (int i = 2; i < argc; i++) {
        manager.addFollower(target, argv[i]);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
                    "Added follower %s to follow %s", argv[i], target.c_str());
    }
    
    manager.spin();
    
    rclcpp::shutdown();
    return 0;
}