// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <algorithm>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/detector_node.hpp"

namespace rm_auto_aim
{
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions & options)
: Node("armor_detector", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");

  // Detection mode: use_yolo parameter
  use_yolo_ = this->declare_parameter("use_yolo", true);
  RCLCPP_INFO(this->get_logger(), "Detection mode: %s", use_yolo_ ? "YOLO" : "Traditional");

  // 声明 detect_color，供 robot 回调动态更新
  if (!this->has_parameter("detect_color")) {
    this->declare_parameter("detect_color", 0);
  }

  // Initialize detector based on mode
  if (use_yolo_) {
    yolo_detector_ = initYoloDetector();
  } else {
    detector_ = initDetector();
  }

  // Armors Publisher
  armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>(
    "/detector/armors", rclcpp::SensorDataQoS());

  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  armor_marker_.ns = "armors";
  armor_marker_.action = visualization_msgs::msg::Marker::ADD;
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.05;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.g = 0.5;
  armor_marker_.color.b = 1.0;
  armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  text_marker_.ns = "classification";
  text_marker_.action = visualization_msgs::msg::Marker::ADD;
  text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker_.scale.z = 0.1;
  text_marker_.color.a = 1.0;
  text_marker_.color.r = 1.0;
  text_marker_.color.g = 1.0;
  text_marker_.color.b = 1.0;
  text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);

  // Debug Publishers
  debug_ = this->declare_parameter("debug", false);
  if (debug_) {
    createDebugPublishers();
  }

  // Performance profiling
  perf_profile_ = this->declare_parameter("perf_profile", false);
  perf_profile_interval_ = this->declare_parameter("perf_profile_interval", 1.0);
  if (perf_profile_) {
    perf_reset();
    perf_last_report_wall_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "Performance profiling ENABLED, interval=%.1fs", perf_profile_interval_);
  }

  // Task subscriber
  is_aim_task_ = true;
  task_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/task_mode", 10, std::bind(&ArmorDetectorNode::taskCallback, this, std::placeholders::_1));

  // Debug param change moniter
  debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  debug_cb_handle_ =
    debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter & p) {
      debug_ = p.as_bool();
      debug_ ? createDebugPublishers() : destroyDebugPublishers();
    });

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
      cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
      pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
      cam_info_sub_.reset();
    });

  // Subscribe to robot info for dynamic detect_color (disabled, use detect_color param only)
  // robot_sub_ = this->create_subscription<vision_interfaces::msg::Robot>(
  //   "/serial_driver/robot", rclcpp::SensorDataQoS(),
  //   [this](vision_interfaces::msg::Robot::ConstSharedPtr msg) {
  //     // foe_color: 0=blue, 1=red → detect_color: RED=0, BLUE=1
  //     // 敌方是蓝色(0)则检测蓝色(BLUE=1)，敌方是红色(1)则检测红色(RED=0)
  //     int new_color = (msg->foe_color == 0) ? 1 : 0;
  //     if (new_color != detect_color_) {
  //       detect_color_ = new_color;
  //       this->set_parameter(rclcpp::Parameter("detect_color", detect_color_));
  //       RCLCPP_INFO(this->get_logger(), "detect_color updated to %s", detect_color_ == 0 ? "RED" : "BLUE");
  //     }
  //   });

  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));
}

void ArmorDetectorNode::taskCallback(const std_msgs::msg::String::SharedPtr task_msg)
{
  std::string task_mode = task_msg->data;
  if (task_mode == "aim") {
    is_aim_task_ = true;
  } else {
    is_aim_task_ = false;
  }
}

void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  auto t_cb_wall_start = std::chrono::steady_clock::now();

  // ---- callback frequency tracking (only when profiling) ----
  if (perf_profile_) {
    if (!perf_first_cb_) {
      // Wall delta
      double wall_dt_ms = std::chrono::duration<double, std::milli>(
        t_cb_wall_start - perf_last_cb_wall_).count();
      perf_wall_dt_sum_ += wall_dt_ms;
      if (wall_dt_ms < perf_wall_dt_min_) perf_wall_dt_min_ = wall_dt_ms;
      if (wall_dt_ms > perf_wall_dt_max_) perf_wall_dt_max_ = wall_dt_ms;

      // Image stamp delta (manual math to avoid rclcpp::Time source mismatch)
      double cur_stamp_s = static_cast<double>(img_msg->header.stamp.sec) +
                           static_cast<double>(img_msg->header.stamp.nanosec) * 1e-9;
      double last_stamp_s = static_cast<double>(perf_last_img_stamp_.sec) +
                            static_cast<double>(perf_last_img_stamp_.nanosec) * 1e-9;
      double img_dt_ms = (cur_stamp_s - last_stamp_s) * 1000.0;
      perf_img_stamp_dt_sum_ += img_dt_ms;
      if (img_dt_ms < perf_img_stamp_dt_min_) perf_img_stamp_dt_min_ = img_dt_ms;
      if (img_dt_ms > perf_img_stamp_dt_max_) perf_img_stamp_dt_max_ = img_dt_ms;

      // Skip hint: flag if img_stamp_dt > 2 * expected (60fps -> ~16.7ms)
      if (img_dt_ms > 33.3) {
        perf_skip_hint_count_++;
      }
    } else {
      perf_first_cb_ = false;
    }
    perf_last_cb_wall_ = t_cb_wall_start;
    perf_last_img_stamp_ = img_msg->header.stamp;
  }

  // ---- detect armors (timed) ----
  auto t_detect_start = std::chrono::steady_clock::now();
  auto armors = detectArmors(img_msg);
  double detect_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t_detect_start).count();

  if (pnp_solver_ != nullptr && is_aim_task_) {
    armors_msg_.header = armor_marker_.header = text_marker_.header = img_msg->header;
    armors_msg_.armors.clear();
    marker_array_.markers.clear();
    armor_marker_.id = 0;
    text_marker_.id = 0;

    // ---- PnP loop (timed) ----
    auto t_pnp_start = std::chrono::steady_clock::now();
    auto_aim_interfaces::msg::Armor armor_msg;
    for (const auto & armor : armors) {
      cv::Mat rvec, tvec;
      bool success = pnp_solver_->solvePnP(armor, rvec, tvec);
      if (success) {
        // Fill basic info
        armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
        armor_msg.number = armor.number;

        // Fill pose
        armor_msg.pose.position.x = tvec.at<double>(0);
        armor_msg.pose.position.y = tvec.at<double>(1);
        armor_msg.pose.position.z = tvec.at<double>(2);
        // rvec to 3x3 rotation matrix
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        // rotation matrix to quaternion
        tf2::Matrix3x3 tf2_rotation_matrix(
          rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
          rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
          rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
          rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
          rotation_matrix.at<double>(2, 2));
        tf2::Quaternion tf2_q;
        tf2_rotation_matrix.getRotation(tf2_q);
        armor_msg.pose.orientation = tf2::toMsg(tf2_q);

        // Fill the distance to image center
        armor_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(armor.center);

        // Fill keypoints
        armor_msg.kpts.clear();
        for (const auto & pt :
             {armor.left_light.top, armor.left_light.bottom, armor.right_light.bottom,
              armor.right_light.top}) {
          geometry_msgs::msg::Point point;
          point.x = pt.x;
          point.y = pt.y;
          armor_msg.kpts.emplace_back(point);
        }

        // Fill the markers
        armor_marker_.id++;
        armor_marker_.scale.y = armor.type == ArmorType::SMALL ? 0.135 : 0.23;
        armor_marker_.pose = armor_msg.pose;
        text_marker_.id++;
        text_marker_.pose.position = armor_msg.pose.position;
        text_marker_.pose.position.y -= 0.1;
        text_marker_.text = armor.classfication_result;
        armors_msg_.armors.emplace_back(armor_msg);
        marker_array_.markers.emplace_back(armor_marker_);
        marker_array_.markers.emplace_back(text_marker_);
      } else {
        RCLCPP_WARN(this->get_logger(), "PnP failed!");
      }
    }
    double pnp_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t_pnp_start).count();

    // ---- publish armors (timed) ----
    auto t_pub_start = std::chrono::steady_clock::now();
    armors_pub_->publish(armors_msg_);
    double pub_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t_pub_start).count();

    // ---- publish markers (timed) ----
    auto t_marker_start = std::chrono::steady_clock::now();
    publishMarkers();
    double marker_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t_marker_start).count();

    // ---- profiling accumulation ----
    if (perf_profile_) {
      int n_armors = static_cast<int>(armors.size());

      perf_detect_total_sum_ += detect_ms;
      if (detect_ms < perf_detect_total_min_) perf_detect_total_min_ = detect_ms;
      if (detect_ms > perf_detect_total_max_) perf_detect_total_max_ = detect_ms;

      perf_pnp_sum_ += pnp_ms;
      if (pnp_ms < perf_pnp_min_) perf_pnp_min_ = pnp_ms;
      if (pnp_ms > perf_pnp_max_) perf_pnp_max_ = pnp_ms;

      perf_publish_sum_ += pub_ms;
      if (pub_ms < perf_publish_min_) perf_publish_min_ = pub_ms;
      if (pub_ms > perf_publish_max_) perf_publish_max_ = pub_ms;

      perf_marker_sum_ += marker_ms;
      if (marker_ms < perf_marker_min_) perf_marker_min_ = marker_ms;
      if (marker_ms > perf_marker_max_) perf_marker_max_ = marker_ms;

      perf_armors_count_sum_ += n_armors;
      if (n_armors < perf_armors_count_min_) perf_armors_count_min_ = n_armors;
      if (n_armors > perf_armors_count_max_) perf_armors_count_max_ = n_armors;

      // total callback time
      double cb_total_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_cb_wall_start).count();
      perf_cb_total_sum_ += cb_total_ms;
      if (cb_total_ms < perf_cb_total_min_) perf_cb_total_min_ = cb_total_ms;
      if (cb_total_ms > perf_cb_total_max_) perf_cb_total_max_ = cb_total_ms;

      perf_cb_count_++;

      // Periodic report
      auto now_wall = std::chrono::steady_clock::now();
      double elapsed_s = std::chrono::duration<double>(now_wall - perf_last_report_wall_).count();
      if (elapsed_s >= perf_profile_interval_) {
        perf_report();
        perf_last_report_wall_ = now_wall;
      }
    }
  }
}

std::unique_ptr<Detector> ArmorDetectorNode::initDetector()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 255;
  int binary_thres = declare_parameter("binary_thres", 160, param_desc);

  param_desc.description = "0-RED, 1-BLUE";
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 1;
  auto detect_color = declare_parameter("detect_color", RED, param_desc);

  Detector::LightParams l_params = {
    .min_ratio = declare_parameter("light.min_ratio", 0.1),
    .max_ratio = declare_parameter("light.max_ratio", 0.4),
    .max_angle = declare_parameter("light.max_angle", 35.0),
    .min_fill_ratio = declare_parameter("light.min_fill_ratio", 0.8),
  };

  Detector::ArmorParams a_params = {
    .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.7),
    .min_small_center_distance = declare_parameter("armor.min_small_center_distance", 0.8),
    .max_small_center_distance = declare_parameter("armor.max_small_center_distance", 3.2),
    .min_large_center_distance = declare_parameter("armor.min_large_center_distance", 3.2),
    .max_large_center_distance = declare_parameter("armor.max_large_center_distance", 5.5),
    .max_angle = declare_parameter("armor.max_angle", 35.0)};

  auto detector = std::make_unique<Detector>(binary_thres, detect_color, l_params, a_params);

  // Init classifier
  auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
  auto model_path = pkg_path + "/model/mlp.onnx";
  auto label_path = pkg_path + "/model/label.txt";
  double threshold = this->declare_parameter("classifier_threshold", 0.7);
  std::vector<std::string> ignore_classes =
    this->declare_parameter("ignore_classes", std::vector<std::string>{"negative"});
  detector->classifier =
    std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);

  return detector;
}

std::unique_ptr<YoloDetector> ArmorDetectorNode::initYoloDetector()
{
  auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
  
  YoloDetector::YoloParams params;
  params.model_path = pkg_path + "/model/" + 
    this->declare_parameter("yolo.model_name", std::string("yolo11.xml"));
  params.device = this->declare_parameter("yolo.device", std::string("CPU"));
  params.input_size = this->declare_parameter("yolo.input_size", 640);
  params.class_num = this->declare_parameter("yolo.class_num", 38);
  params.score_threshold = this->declare_parameter("yolo.score_threshold", 0.7);
  params.nms_threshold = this->declare_parameter("yolo.nms_threshold", 0.3);
  params.min_confidence = this->declare_parameter("yolo.min_confidence", 0.8);
  params.use_roi = this->declare_parameter("yolo.use_roi", false);
  
  if (params.use_roi) {
    params.roi.x = this->declare_parameter("yolo.roi.x", 0);
    params.roi.y = this->declare_parameter("yolo.roi.y", 0);
    params.roi.width = this->declare_parameter("yolo.roi.width", -1);
    params.roi.height = this->declare_parameter("yolo.roi.height", -1);
  }
  
  RCLCPP_INFO(this->get_logger(), "YOLO model path: %s", params.model_path.c_str());
  RCLCPP_INFO(this->get_logger(), "YOLO device: %s", params.device.c_str());
  RCLCPP_INFO(this->get_logger(), "YOLO input size: %d", params.input_size);
  
  return std::make_unique<YoloDetector>(params);
}

std::vector<Armor> ArmorDetectorNode::detectArmors(
  const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
{
  // Convert ROS img to cv::Mat
  auto t_bridge_start = std::chrono::steady_clock::now();
  auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
  double bridge_ms = 0, flip_ms = 0, cvt_ms = 0, yolo_ms = 0;
  if (perf_profile_) {
    bridge_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t_bridge_start).count();
  }

  // Flip image if needed
  auto t_flip_start = std::chrono::steady_clock::now();
  cv::flip(img, img, -1);
  if (perf_profile_) {
    flip_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t_flip_start).count();
  }

  std::vector<Armor> armors;
  auto final_time = this->now();
  double latency = 0;

  if (use_yolo_) {
    // YOLO detection mode
    // Note: YOLO expects BGR, but we have RGB from ROS, convert it
    auto t_cvt_start = std::chrono::steady_clock::now();
    cv::Mat bgr_img;
    cv::cvtColor(img, bgr_img, cv::COLOR_RGB2BGR);
    if (perf_profile_) {
      cvt_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_cvt_start).count();
    }

    auto t_yolo_start = std::chrono::steady_clock::now();
    armors = yolo_detector_->detect(bgr_img);
    if (perf_profile_) {
      yolo_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_yolo_start).count();
    }
    
    // 按 detect_color 过滤颜色（与传统检测一致）
    int cur_detect_color = get_parameter("detect_color").as_int();
    armors.erase(
      std::remove_if(armors.begin(), armors.end(),
        [cur_detect_color](const Armor & a) {
          return a.left_light.color != cur_detect_color || a.right_light.color != cur_detect_color;
        }),
      armors.end());

    latency = (this->now() - img_msg->header.stamp).seconds() * 1000;
    RCLCPP_DEBUG_STREAM(this->get_logger(), "YOLO Latency: " << latency << "ms");
    
    // Debug visualization for YOLO
    if (debug_) {
      yolo_detector_->drawResults(img, armors);
      // Draw camera center
      cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
      // Draw latency
      std::stringstream latency_ss;
      latency_ss << "YOLO Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
      cv::putText(img, latency_ss.str(), cv::Point(10, 30), 
                  cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
      result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
    }
  } else {
    // Traditional detection mode
    // Update params
    detector_->binary_thres = get_parameter("binary_thres").as_int();
    detector_->detect_color = get_parameter("detect_color").as_int();
    detector_->classifier->threshold = get_parameter("classifier_threshold").as_double();

    armors = detector_->detect(img);

    latency = (this->now() - img_msg->header.stamp).seconds() * 1000;
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Traditional Latency: " << latency << "ms");

    // Publish debug info for traditional mode
    if (debug_) {
      binary_img_pub_.publish(
        cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img).toImageMsg());

      // Sort lights and armors data by x coordinate
      std::sort(
        detector_->debug_lights.data.begin(), detector_->debug_lights.data.end(),
        [](const auto & l1, const auto & l2) { return l1.center_x < l2.center_x; });
      std::sort(
        detector_->debug_armors.data.begin(), detector_->debug_armors.data.end(),
        [](const auto & a1, const auto & a2) { return a1.center_x < a2.center_x; });

      lights_data_pub_->publish(detector_->debug_lights);
      armors_data_pub_->publish(detector_->debug_armors);

      if (!armors.empty()) {
        auto all_num_img = detector_->getAllNumbersImage();
        number_img_pub_.publish(
          *cv_bridge::CvImage(img_msg->header, "mono8", all_num_img).toImageMsg());
      }

      detector_->drawResults(img);
      // Draw camera center
      cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
      // Draw latency
      std::stringstream latency_ss;
      latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
      cv::putText(img, latency_ss.str(), cv::Point(10, 30), 
                  cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
      result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
    }
  }

  // Accumulate internal timing breakdown
  if (perf_profile_) {
    perf_bridge_sum_ += bridge_ms;
    if (bridge_ms < perf_bridge_min_) perf_bridge_min_ = bridge_ms;
    if (bridge_ms > perf_bridge_max_) perf_bridge_max_ = bridge_ms;

    perf_flip_sum_ += flip_ms;
    if (flip_ms < perf_flip_min_) perf_flip_min_ = flip_ms;
    if (flip_ms > perf_flip_max_) perf_flip_max_ = flip_ms;

    perf_cvtcolor_sum_ += cvt_ms;
    if (cvt_ms < perf_cvtcolor_min_) perf_cvtcolor_min_ = cvt_ms;
    if (cvt_ms > perf_cvtcolor_max_) perf_cvtcolor_max_ = cvt_ms;

    perf_yolo_sum_ += yolo_ms;
    if (yolo_ms < perf_yolo_min_) perf_yolo_min_ = yolo_ms;
    if (yolo_ms > perf_yolo_max_) perf_yolo_max_ = yolo_ms;
  }

  return armors;
}

void ArmorDetectorNode::createDebugPublishers()
{
  lights_data_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::DebugLights>("/detector/debug_lights", 10);
  armors_data_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::DebugArmors>("/detector/debug_armors", 10);

  binary_img_pub_ = image_transport::create_publisher(this, "/detector/binary_img");
  number_img_pub_ = image_transport::create_publisher(this, "/detector/number_img");
  result_img_pub_ = image_transport::create_publisher(this, "/detector/result_img");
}

void ArmorDetectorNode::destroyDebugPublishers()
{
  lights_data_pub_.reset();
  armors_data_pub_.reset();

  binary_img_pub_.shutdown();
  number_img_pub_.shutdown();
  result_img_pub_.shutdown();
}

void ArmorDetectorNode::publishMarkers()
{
  using Marker = visualization_msgs::msg::Marker;
  armor_marker_.action = armors_msg_.armors.empty() ? Marker::DELETE : Marker::ADD;
  marker_array_.markers.emplace_back(armor_marker_);
  marker_pub_->publish(marker_array_);
}

void ArmorDetectorNode::perf_reset()
{
  perf_cb_count_ = 0;
  perf_skip_hint_count_ = 0;

  perf_img_stamp_dt_sum_ = 0; perf_img_stamp_dt_min_ = 1e9; perf_img_stamp_dt_max_ = 0;
  perf_wall_dt_sum_ = 0; perf_wall_dt_min_ = 1e9; perf_wall_dt_max_ = 0;
  perf_cb_total_sum_ = 0; perf_cb_total_min_ = 1e9; perf_cb_total_max_ = 0;
  perf_detect_total_sum_ = 0; perf_detect_total_min_ = 1e9; perf_detect_total_max_ = 0;
  perf_yolo_sum_ = 0; perf_yolo_min_ = 1e9; perf_yolo_max_ = 0;
  perf_bridge_sum_ = 0; perf_bridge_min_ = 1e9; perf_bridge_max_ = 0;
  perf_flip_sum_ = 0; perf_flip_min_ = 1e9; perf_flip_max_ = 0;
  perf_cvtcolor_sum_ = 0; perf_cvtcolor_min_ = 1e9; perf_cvtcolor_max_ = 0;
  perf_pnp_sum_ = 0; perf_pnp_min_ = 1e9; perf_pnp_max_ = 0;
  perf_publish_sum_ = 0; perf_publish_min_ = 1e9; perf_publish_max_ = 0;
  perf_marker_sum_ = 0; perf_marker_min_ = 1e9; perf_marker_max_ = 0;
  perf_armors_count_sum_ = 0; perf_armors_count_min_ = 9999; perf_armors_count_max_ = 0;
}

void ArmorDetectorNode::perf_report()
{
  if (perf_cb_count_ == 0) return;

  double n = static_cast<double>(perf_cb_count_);
  double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - perf_last_report_wall_).count();
  double cb_fps = elapsed > 0 ? n / elapsed : 0;

  auto avg = [](double sum, double c) { return c > 0 ? sum / c : 0; };

  std::stringstream ss;
  ss << "\n[detector_profile]";
  ss << "\ncb_fps=" << std::fixed << std::setprecision(1) << cb_fps;
  ss << "\nimg_stamp_dt_avg=" << std::fixed << std::setprecision(1) << avg(perf_img_stamp_dt_sum_, n)
     << "ms  min=" << perf_img_stamp_dt_min_ << "  max=" << perf_img_stamp_dt_max_;
  ss << "\nwall_dt_avg=" << std::fixed << std::setprecision(1) << avg(perf_wall_dt_sum_, n)
     << "ms  min=" << perf_wall_dt_min_ << "  max=" << perf_wall_dt_max_;
  ss << "\ncallback_total_avg=" << std::fixed << std::setprecision(1) << avg(perf_cb_total_sum_, n)
     << "ms  min=" << perf_cb_total_min_ << "  max=" << perf_cb_total_max_;
  ss << "\ndetect_total_avg=" << std::fixed << std::setprecision(1) << avg(perf_detect_total_sum_, n)
     << "ms  min=" << perf_detect_total_min_ << "  max=" << perf_detect_total_max_;
  ss << "\n  bridge_avg=" << std::fixed << std::setprecision(1) << avg(perf_bridge_sum_, n)
     << "ms  flip_avg=" << avg(perf_flip_sum_, n)
     << "ms  cvtcolor_avg=" << avg(perf_cvtcolor_sum_, n) << "ms";
  ss << "\n  yolo_avg=" << std::fixed << std::setprecision(1) << avg(perf_yolo_sum_, n)
     << "ms  min=" << perf_yolo_min_ << "  max=" << perf_yolo_max_;
  ss << "\npnp_avg=" << std::fixed << std::setprecision(1) << avg(perf_pnp_sum_, n)
     << "ms  min=" << perf_pnp_min_ << "  max=" << perf_pnp_max_;
  ss << "\npublish_avg=" << std::fixed << std::setprecision(3) << avg(perf_publish_sum_, n)
     << "ms  min=" << perf_publish_min_ << "  max=" << perf_publish_max_;
  ss << "\nmarker_avg=" << std::fixed << std::setprecision(3) << avg(perf_marker_sum_, n)
     << "ms  min=" << perf_marker_min_ << "  max=" << perf_marker_max_;
  ss << "\narmors_avg=" << std::fixed << std::setprecision(1) << avg(perf_armors_count_sum_, n)
     << "  min=" << perf_armors_count_min_ << "  max=" << perf_armors_count_max_;
  ss << "\ndropped_or_skipped_hint: skipped_frames=" << perf_skip_hint_count_
     << " (img_stamp_dt > 33.3ms, i.e. >2x 60fps period)";

  RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

  perf_reset();
  perf_last_report_wall_ = std::chrono::steady_clock::now();
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)
