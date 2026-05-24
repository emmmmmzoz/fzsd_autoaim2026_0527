// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.
// YOLO Detector for ROS2 - Adapted from sp_vision_25

#ifndef ARMOR_DETECTOR__YOLO_DETECTOR_HPP_
#define ARMOR_DETECTOR__YOLO_DETECTOR_HPP_

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"

namespace rm_auto_aim
{

// YOLO检测结果结构体
struct YoloArmor
{
  int class_id;
  float confidence;
  cv::Rect box;
  std::vector<cv::Point2f> keypoints;  // 4个角点: 左上、右上、右下、左下
  
  // 解析后的属性
  int color;      // 0: blue, 1: red
  std::string number;
  ArmorType type;
};

class YoloDetector
{
public:
  struct YoloParams
  {
    std::string model_path;
    std::string device = "CPU";
    int input_size = 640;        // YOLO11用640, YOLOv8用416
    int class_num = 38;          // 类别数
    float score_threshold = 0.7;
    float nms_threshold = 0.3;
    float min_confidence = 0.8;
    bool use_roi = false;
    cv::Rect roi;
  };

  explicit YoloDetector(const YoloParams & params);
  
  // 检测装甲板，返回Armor列表（兼容原有接口）
  std::vector<Armor> detect(const cv::Mat & img);
  
  // 调试用：绘制检测结果
  void drawResults(cv::Mat & img, const std::vector<Armor> & armors);

private:
  YoloParams params_;
  
  ov::Core core_;
  ov::CompiledModel compiled_model_;
  
  cv::Point2f offset_;
  
  // 38类装甲板属性映射表
  // {color, name, type}: 0=blue, 1=red, 2=extinguish
  static const std::vector<std::tuple<int, std::string, ArmorType>> armor_classes_;
  
  // 预处理
  cv::Mat preprocess(const cv::Mat & img, double & scale);
  
  // 后处理：解析YOLO输出
  std::vector<YoloArmor> postprocess(double scale, const ov::Tensor & output_tensor);
  
  // 将YoloArmor转换为Armor
  Armor convertToArmor(const YoloArmor & yolo_armor);
  
  // 对关键点排序：左上、右上、右下、左下
  void sortKeypoints(std::vector<cv::Point2f> & keypoints);
  
  // 检查装甲板是否有效
  bool checkArmor(const YoloArmor & armor);
  
  // 从class_id解析颜色、数字、类型
  void parseClassId(int class_id, int & color, std::string & number, ArmorType & type);
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__YOLO_DETECTOR_HPP_
