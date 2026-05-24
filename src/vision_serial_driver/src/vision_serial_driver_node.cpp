#include "../include/vision_serial_driver/vision_serial_driver_node.hpp"

serial_driver_node::serial_driver_node(std::string device_name, std::string node_name)
    : rclcpp::Node(node_name), vArray{new visionArray}, rArray{new robotArray},
      dev_name{new std::string(device_name)},
      portConfig{new SerialPortConfig(115200, FlowControl::NONE, Parity::NONE, StopBits::ONE)}, ctx{IoContext(2)}
{
  RCLCPP_INFO(get_logger(), "节点:/%s启动", node_name.c_str());
  muzzleSpeedFilter.Size=10;
  // 清零数组
  memset(vArray->array, 0, sizeof(visionArray));
  memset(rArray->array, 0, sizeof(robotArray));


  // 设置重启计时器1hz.
  reopenTimer = create_wall_timer(
      1s, std::bind(&serial_driver_node::serial_reopen_callback, this));

  // 设置发布计时器500hz.
  publishTimer = create_wall_timer(
      2ms, std::bind(&serial_driver_node::robot_callback, this));

  // TF broadcaster
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.00591);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  VelControl = this->declare_parameter("vel_control", 0);
  color =  this->declare_parameter("my_color", 0);


  // Detect parameter client

  // 发布Robot信息.
  publisher = create_publisher<vision_interfaces::msg::Robot>(
      "/serial_driver/robot", rclcpp::SensorDataQoS());
  gameStatePublisher = create_publisher<vision_interfaces::msg::GameState>(
    "/serial_driver/game_state", rclcpp::SensorDataQoS());

    //订阅导航信息.
  navSub = create_subscription<geometry_msgs::msg::Twist>(
      "/red_standard_robot1/cmd_vel",rclcpp::SensorDataQoS(),std::bind(&serial_driver_node::nav_callback, this, std::placeholders::_1));

  // 订阅AutoAim信息.
  autoAimSub = create_subscription<vision_interfaces::msg::AutoAim>(
      "/serial_driver/aim_target", rclcpp::SensorDataQoS(), std::bind(&serial_driver_node::auto_aim_callback, this, std::placeholders::_1));


  navpublisher = create_publisher<std_msgs::msg::UInt8>(
      "/nav2_command", rclcpp::SensorDataQoS());

  aimpublisher = create_publisher<vision_interfaces::msg::Robot>(
      "/aim_target_msg", rclcpp::SensorDataQoS());

  writeTimer = create_wall_timer(
    2ms, std::bind(&serial_driver_node::serial_write_callback, this));

  



  //设置串口读取线程.
  serialReadThread = std::thread(&serial_driver_node::serial_read_thread, this);
  serialReadThread.detach();
}

serial_driver_node::~serial_driver_node()
{
  if (serialDriver.port()->is_open())
  {
    serialDriver.port()->close();
  }
}

void serial_driver_node::serial_reopen_callback()
{
  // 串口失效时尝试重启
  if (!isOpen)
  {
    try
    {
      RCLCPP_WARN(get_logger(), "重启串口:%s...", dev_name->c_str());
      serialDriver.init_port(*dev_name, *portConfig);
      serialDriver.port()->open();
      isOpen = serialDriver.port()->is_open();
    }
    catch (const std::system_error &error)
    {
      RCLCPP_ERROR(get_logger(), "打开串口:%s失败", dev_name->c_str());
      isOpen = false;
    }
    if (isOpen)
      RCLCPP_INFO(get_logger(), "打开串口:%s成功", dev_name->c_str());
  }
}

void serial_driver_node::serial_read_thread()
{
  while (rclcpp::ok())
  {
    std::vector<uint8_t> head(2);
    std::vector<uint8_t> robotData(sizeof(rArray->array) - 2);
    if (isOpen)
    {
      try
      {
        serialDriver.port()->receive(head);
        if (head[0] == 0xA5 && head[1] == 0x00)
        { // 包头为0xA5
          serialDriver.port()->receive(robotData);
          robotData.resize(sizeof(rArray->array));
          robotData.insert(robotData.begin(), head[1]);
          robotData.insert(robotData.begin(), head[0]);
          float lastSpeed = rArray->msg.muzzleSpeed;
          memcpy(rArray->array, robotData.data(), sizeof(rArray->array));
          rArray->msg.muzzleSpeed = rArray->msg.muzzleSpeed > 15 ? rArray->msg.muzzleSpeed : 15.0;
          if(rArray->msg.muzzleSpeed != lastSpeed)muzzleSpeedFilter.update(rArray->msg.muzzleSpeed);
        }
      }
      catch (const std::exception &error)
      {
        RCLCPP_ERROR(get_logger(), "读取串口时发生错误.");
        isOpen = false;
      }
    }
  }
}

void serial_driver_node::serial_write(uint8_t *data, size_t len)
{
  std::vector<uint8_t> tempData(data, data + len);
  try
  {
    serialDriver.port()->send(tempData);
    // RCLCPP_INFO(get_logger(), "写入串口.");
  }
  catch (const std::exception &error)
  {
    RCLCPP_ERROR(get_logger(), "写入串口时发生错误.");
    isOpen = false;
  }
}


void serial_driver_node::serial_write_callback()
{
  if (isOpen)
  {
    vArray->msg.head = 0xA5;
    serial_write(vArray->array, sizeof(vArray->array));
  }
  else
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "串口未打开, 无法发送数据");
  }
}


void serial_driver_node::nav_callback(const geometry_msgs::msg::Twist::SharedPtr Msg) {
  if (isOpen)
  {
    // 这里将导航速度方向取反，以匹配下位机/协议要求
    vArray->msg.vx = -Msg->linear.x;
    vArray->msg.vy = -Msg->linear.y;
    vArray->msg.wz = Msg->angular.z;
  }
}

void serial_driver_node::auto_aim_callback(const vision_interfaces::msg::AutoAim vMsg)
{
 if (isOpen)
  {

    vArray->msg.fire = vMsg.fire;
    vArray->msg.gyroscope = vMsg.gyroscope;
    vArray->msg.aimPitch = vMsg.aim_pitch;  // 度
    vArray->msg.aimYaw = vMsg.aim_yaw;  // 度
    vArray->msg.tracking = vMsg.tracking;
    vArray->msg.aimPVel = vMsg.aim_p_vel;  // 度/秒
    vArray->msg.aimYVel = vMsg.aim_y_vel;  // 度/秒
    vArray->msg.vel_control = VelControl;
    vArray->msg.head = 0xA5;

    auto aimmsg = vision_interfaces::msg::Robot();
    aimmsg.self_pitch = vMsg.aim_pitch;
    aimmsg.self_yaw = vMsg.aim_yaw;
    aimpublisher->publish(aimmsg);



    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "aimYaw: %.2f, aimPitch: %.2f, selfYaw: %.2f, selfPitch: %.2f",
      vMsg.aim_yaw, vMsg.aim_pitch, rArray->msg.robotYaw, rArray->msg.robotPitch);

    serial_write(vArray->array, sizeof(vArray->array));
  }
}

void serial_driver_node::robot_callback()
{
  if (isOpen)
  {
    try
    {
      auto msg = vision_interfaces::msg::Robot();
      auto game_state_msg = vision_interfaces::msg::GameState();
      auto state_msg = std_msgs::msg::UInt8();
      msg.mode = rArray->msg.mode;
      // msg.foe_color = rArray->msg.foeColor;// 使用下位机发来的敌方颜色 0-blue 1-red(我方颜色)
      msg.foe_color = 1;// 使用下位机发来的敌方颜色 0-blue 1-red
      msg.self_yaw = rArray->msg.robotYaw;
      msg.self_pitch = rArray->msg.robotPitch;

      state_msg.data = 0;  // data field removed from protocol
      
      double muzzle_speed = rArray->msg.muzzleSpeed;
      //muzzleSpeedFilter.get_avg(muzzle_speed);
      msg.muzzle_speed = muzzle_speed;
      if (msg.muzzle_speed < 12.0)
      {
        msg.muzzle_speed = 20.0;
      }

  game_state_msg.current_robot_quantity = rArray->msg.current_robot_quantity;
  game_state_msg.blood_warn_state = rArray->msg.blood_warn_state;
  game_state_msg.game_process = rArray->msg.game_process;
  game_state_msg.current_state_left_time = rArray->msg.current_state_left_time;
  game_state_msg.is_success_ourpreempt = rArray->msg.is_success_ourpreempt;
  game_state_msg.is_success_enemypreempt = rArray->msg.is_success_enemypreempt;

    
      publisher->publish(msg);
      

  gameStatePublisher->publish(game_state_msg);
      navpublisher->publish(state_msg);
      geometry_msgs::msg::TransformStamped t;
      timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
      t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
      t.header.frame_id = "aim_odom";
      t.child_frame_id = "aim_gimbal_link";
      tf2::Quaternion q;
      q.setRPY(0, rArray->msg.robotPitch*3.1415926535/180.0, rArray->msg.robotYaw*3.1415926535/180.0);
      t.transform.rotation = tf2::toMsg(q);
      tf_broadcaster_->sendTransform(t);
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 20, "处理串口数据时发生错误: %s", ex.what());
    }
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::string dev_name = "/dev/ttyACM0";
  std::shared_ptr<serial_driver_node> node = std::make_shared<serial_driver_node>(dev_name, "vision_serial_driver");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
