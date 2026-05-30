// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__DETECTOR_NODE_HPP_

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/detector.hpp"
#include "armor_detector/number_classifier.hpp"
#include "armor_detector/pnp_solver.hpp"
#include "armor_detector/yolo_detector.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
// #include "vision_interfaces/msg/robot.hpp"

namespace rm_auto_aim
{

class ArmorDetectorNode : public rclcpp::Node
{
public:
  ArmorDetectorNode(const rclcpp::NodeOptions & options);

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

  std::unique_ptr<Detector> initDetector();
  std::unique_ptr<YoloDetector> initYoloDetector();
  std::vector<Armor> detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);

  void createDebugPublishers();
  void destroyDebugPublishers();

  void publishMarkers();

  //  task subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
  bool is_aim_task_;
  void taskCallback(const std_msgs::msg::String::SharedPtr task_msg);

  // Detection mode: true = YOLO, false = traditional
  bool use_yolo_;

  // Armor Detector (traditional)
  std::unique_ptr<Detector> detector_;
  
  // YOLO Detector
  std::unique_ptr<YoloDetector> yolo_detector_;

  // Detected armors publisher
  auto_aim_interfaces::msg::Armors armors_msg_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker armor_marker_;
  visualization_msgs::msg::Marker text_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Camera info part
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  cv::Point2f cam_center_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
  std::unique_ptr<PnPSolver> pnp_solver_;

  // Image subscrpition
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  // Robot color subscription (for dynamic detect_color update) - disabled, use detect_color param only
  // rclcpp::Subscription<vision_interfaces::msg::Robot>::SharedPtr robot_sub_;
  // int detect_color_{0};  // 0=RED, 1=BLUE

  // Debug information
  bool debug_;
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;
  image_transport::Publisher binary_img_pub_;
  image_transport::Publisher number_img_pub_;
  image_transport::Publisher result_img_pub_;

  // Performance profiling
  bool perf_profile_;
  double perf_profile_interval_;
  int64_t perf_cb_count_;
  std::chrono::steady_clock::time_point perf_last_report_wall_;
  std::chrono::steady_clock::time_point perf_last_cb_wall_;
  builtin_interfaces::msg::Time perf_last_img_stamp_;
  bool perf_first_cb_;

  double perf_img_stamp_dt_sum_, perf_img_stamp_dt_min_, perf_img_stamp_dt_max_;
  double perf_wall_dt_sum_, perf_wall_dt_min_, perf_wall_dt_max_;
  double perf_cb_total_sum_, perf_cb_total_min_, perf_cb_total_max_;
  double perf_detect_total_sum_, perf_detect_total_min_, perf_detect_total_max_;
  double perf_yolo_sum_, perf_yolo_min_, perf_yolo_max_;
  double perf_bridge_sum_, perf_bridge_min_, perf_bridge_max_;
  double perf_flip_sum_, perf_flip_min_, perf_flip_max_;
  double perf_cvtcolor_sum_, perf_cvtcolor_min_, perf_cvtcolor_max_;
  double perf_pnp_sum_, perf_pnp_min_, perf_pnp_max_;
  double perf_publish_sum_, perf_publish_min_, perf_publish_max_;
  double perf_marker_sum_, perf_marker_min_, perf_marker_max_;
  double perf_armors_count_sum_;
  int perf_armors_count_min_, perf_armors_count_max_;
  int perf_skip_hint_count_;

  void perf_report();
  void perf_reset();
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_NODE_HPP_
