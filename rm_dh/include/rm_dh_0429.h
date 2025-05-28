#ifndef Dh_Gripper_H
#define Dh_Gripper_H

#include "rclcpp/rclcpp.hpp"
#include "rm_define.h"
#include "rm_service.h"
#include "sensor_msgs/msg/joy.hpp"
#include <iostream>
#include <sensor_msgs/msg/joint_state.hpp>
#include "geometry_msgs/msg/pose.hpp"

/* 使用变长数组 */
#include <vector>
#include <algorithm>

using namespace std;

class Dh_Gripper : public rclcpp::Node
{
public:

    explicit Dh_Gripper(std::string name);
    ~Dh_Gripper()
    {
        Rm_Api_.Service_Arm_Socket_Close(m_sockhand_);
        if (joint_state_thread_.joinable())
        {
            joint_state_thread_.join();
        }
        if (pose_thread_.joinable())
        {
            pose_thread_.join();
        }
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    int MoveJoint(float Step, u_int32_t joint_id);
    int MovePose(Pose pose, float *joint);
    void JointLimit();
    void publish_joint_states();
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_pose_;
    sensor_msgs::msg::JointState joint_state_msg_;
    geometry_msgs::msg::Pose pose_msg_;
    

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    int arm_type_ = 65;
    bool follow_ = false;

    int rm_port_ = 1;                     // 通讯端口，0-控制器RS485端口，1-末端接口板RS485接口，3-控制器ModbusTCP设备
    int dh_addr_ = 1;                     // 夹爪地址
    int dh_baudrate_ = 115200;            // 夹爪波特率
    int dh_init_addr_ = 256;              // 初始化夹爪 写入1：回零位
    int dh_effort_addr_ = 50;            // 力值 20-100，百分比
    int dh_position_addr_ = 259;          // 位置 0-1000，千分比
    int dh_speed_addr_ = 50;             // 速度 1-100，百分比
    int dh_position_feedback_addr_ = 514; // 速度 1-100，百分比
    int dh_speed_ = 20;
    bool block_ = RM_BLOCK;               // 阻塞：RM_BLOCK，不阻塞：RM_NONBLOCK
    int coils_data_ = 0;                  // 读取寄存器的值
    u_int32_t dh_Step_ = 20;              // 大寰夹爪步进
    float rm_Step_ = 0.5;                 // 睿尔曼机械臂步进

    float joint_[7];                      // 关节角度数组
    float speed_[7];                      // 关节速度数组
    unsigned char v_ = 60;                // 速度比例1~100，即规划速度和加速度占关节最大线转速和加速度的比例
    unsigned char r_ = 0;                 // 轨迹交融半径，暂不支持交融，目前默认0
    int trajectory_connect_ = 0;          // 代表是否和下一条运动一起规划，0 代表立即规划，1 代表和下一条轨迹一起规划，当为1 时，轨迹不会立即执行
    RM_Service Rm_Api_;                   // api类
    SOCKHANDLE m_sockhand_ = -1;          //机械臂TCp网络通信套接字
    const char *tcp_ip_ = "192.168.37.18";// tcp ip
    int tcp_port_ = 8080;                 // tcp port
    float joint_read_[7];                 // 关节读取数组
    std::thread joint_state_thread_;      //joint_state线程
    std::thread pose_thread_;             //pose线程
    void publish_joint_states_thread();   //发布joint_states线程函数
    Pose pose_;                           //pose结构体
    Pose init_pose_;                      //pose结构体
    uint16_t Arm_Err;                     //机械臂运行错误代码
    uint16_t Sys_Err;                     //控制器错误代码
    FRAME frame_;                         //定义坐标系数组
    Euler eu_;                            //欧拉角结构体
    Quat quat_;                           //四元数结构体
};

#endif // Dh_Gripper_H

