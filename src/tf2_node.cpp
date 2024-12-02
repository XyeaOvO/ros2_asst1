#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/pose.hpp"
#include "turtlesim/msg/pose.hpp"
#include <vector>
#include <memory>

class TFNode: public rclcpp::Node
{
private:
    tf2_ros::TransformBroadcaster tfb;
    std::string turtleName;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = turtleName;
        t.transform.translation.x = msg->x;
        t.transform.translation.y = msg->y;
        t.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, msg->theta);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tfb.sendTransform(t);
    }

public:
    TFNode(const std::string& st)
        : Node(st + "_broadcaster"), tfb(this), turtleName(st)
    {
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            turtleName + "/pose", 10,
            std::bind(&TFNode::pose_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Started TF broadcaster for %s", turtleName.c_str());
    }

    ~TFNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down TF broadcaster for %s", turtleName.c_str());
    }
};

class MultiTFManager 
{
private:
    std::vector<std::shared_ptr<TFNode>> nodes;
    rclcpp::executors::MultiThreadedExecutor executor;

public:
    void addTurtle(const std::string& turtleName) {
        nodes.push_back(std::make_shared<TFNode>(turtleName));
        executor.add_node(nodes.back());
    }

    void spin() {
        executor.spin();
    }
};

int main(int argc, char *argv[])
{
    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                     "Usage: tf turtle1_name [turtle2_name turtle3_name ...]");
        return 1;
    }

    rclcpp::init(argc, argv);
    
    MultiTFManager manager;
    
    // Add all turtles specified in command line arguments
    for (int i = 1; i < argc; ++i) {
        manager.addTurtle(argv[i]);
    }
    
    // Spin all nodes
    manager.spin();
    
    rclcpp::shutdown();
    return 0;
}