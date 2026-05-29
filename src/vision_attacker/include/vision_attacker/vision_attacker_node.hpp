#pragma once
#include <rclcpp/rclcpp.hpp>
#include <vision_interfaces/msg/robot.hpp>
#include <vision_interfaces/msg/auto_aim.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include "auto_aim_interfaces/msg/target.hpp"
#include "vision_attacker/outpost.hpp"
#include <Eigen/Dense>
#include "vision_attacker/target.hpp"
#include "vision_attacker/planner.hpp"
#include <chrono>
#include <vector>


class Tracker_node : public rclcpp::Node
{
public:
    explicit Tracker_node(std::string node_name);


private:
    void robot_callback(const vision_interfaces::msg::Robot robot);
    void target_callback(const auto_aim_interfaces::msg::Target target_msg);

    std::unique_ptr<auto_aim::Planner> planner_;
    double lagTime;
    double airK;
    double yawFix;
    double pitchFix;
    double pitchFixPerMeter;
    double pitchFixMin;
    double thresholdFix;
    double carThreshold;
    double yawThreshold;
    double pitchThreshold;
    double outpostThreshold;

    
    visualization_msgs::msg::Marker aimPoint;
    visualization_msgs::msg::Marker realAimPoint;
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr targetSub;
    std::unique_ptr<vision_interfaces::msg::Robot> robotPtr;
    rclcpp::Subscription<vision_interfaces::msg::Robot>::SharedPtr robotSub;
    rclcpp::Publisher<vision_interfaces::msg::AutoAim>::SharedPtr aimPub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr aimMarkerPub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr angleErrorPub;
};
