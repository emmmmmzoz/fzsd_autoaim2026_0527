#ifndef SERIAL_PACKET_H
#define SERIAL_PACKET_H

#include <cstdint>
#include <algorithm>

#pragma pack(1)

#define visionMsg inf_visionMsg
#define robotMsg inf_robotMsg
//#define VelMsg Vel_visionMsg

struct inf_visionMsg
{
    uint16_t head;
    uint8_t fire;     // 开火标志
    uint8_t tracking; // 跟踪标志
    uint8_t vel_control; // 速度控制标志
    float aimYaw;     // 目标Yaw
    float aimPitch;   // 目标Pitch
    float aimPVel;      // 目标Pitch速度
    float aimYVel;      // 目标Yaw速度
    float vx;
    float vy;
    float wz;
    uint8_t gyroscope;
};

// struct Vel_visionMsg
// {
//     uint16_t head;
//     uint8_t fire;     // 开火标志
//     uint8_t tracking; // 跟踪标志
//     uint8_t VelControl; // 速度控制标志
//     float aimYaw;     // 目标Yaw
//     float aimPitch;   // 目标Pitch
//     float aimPVel;      // 目标Pitch速度
//     float aimYVel;      // 目标Yaw速度
// };

struct inf_robotMsg
{
    uint16_t head;
    uint8_t mode;
    uint8_t foeColor;  // 敌方颜色 0-blue 1-red
    float robotYaw;    // 自身Yaw
    float robotPitch;  // 自身Pitch
    float muzzleSpeed; // 弹速
    float bigYaw;      // 大Yaw
    int current_robot_quantity; //当前机器人存活数量
    int blood_warn_state;  //血量值
    int game_process; //比赛进程状态（4=比赛进行时）
    float current_state_left_time;  //当前阶段剩余时间
    uint8_t is_success_ourpreempt; //我方成功站点标志（0/1）
    uint8_t is_success_enemypreempt; //敌方成功站点标志（0/1）
};

union visionArray
{
    struct visionMsg msg;
    uint8_t array[sizeof(struct visionMsg)];
};

// union VelArray
// {
//     struct VelMsg msg;
//     uint8_t array[sizeof(struct VelMsg)];
// };

union robotArray
{
    struct robotMsg msg;
    uint8_t array[sizeof(struct robotMsg)];
};

#pragma pack()

#endif // SERIAL_PACKET_H
