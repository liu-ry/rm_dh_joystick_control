#include "rm_dh.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>
#include <jsoncpp/json/json.h>

using namespace std;

/* 构造函数 */
Dh_Gripper::Dh_Gripper(std::string name) : Node(name)
{
    using namespace std::placeholders;
    u_int32_t res;

    // 创建订阅者，自动注册到节点
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "pico_driver/pose",                                               // 话题名称
        10,                                                                 // 队列大小
        std::bind(&Dh_Gripper::joyCallback, this, std::placeholders::_1)); // 设置回调函数
    
        // 创建一个发布者，发布到"rm_dh_joint_states"话题，队列大小为10
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("rm_joint_topic", 10);
    joint_state_msg_.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "finger1_joint"};
    joint_state_msg_.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 设置关节位置
    joint_state_msg_.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 设置关节速度
    joint_action_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("rm_action_joint_topic", 10);
    joint_action_state_msg_.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "finger1_joint"};
    joint_action_state_msg_.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 设置关节位置
    joint_action_state_msg_.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 设置关节速度
    
    // 创建一个发布者，发布到"pose_topic"话题，队列大小为10
    publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("rm_pose_topic", 10);
    // 设置机械臂的位置（以米为单位）
    pose_msg_.position.x = 0.0;
    pose_msg_.position.y = 0.0;
    pose_msg_.position.z = 0.0;
    // 设置机械臂的姿态（以四元数表示）
    pose_msg_.orientation.x = 0.0;
    pose_msg_.orientation.y = 0.0;
    pose_msg_.orientation.z = 0.0;
    pose_msg_.orientation.w = 0.0;
    action_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("rm_action_pose_topic", 10);
    // 设置机械臂的位置（以米为单位）
    action_pose_msg_.position.x = 0.0;
    action_pose_msg_.position.y = 0.0;
    action_pose_msg_.position.z = 0.0;
    // 设置机械臂的姿态（以四元数表示）
    action_pose_msg_.orientation.x = 0.0;
    action_pose_msg_.orientation.y = 0.0;
    action_pose_msg_.orientation.z = 0.0;
    action_pose_msg_.orientation.w = 0.0;

    // SDK初始化
    Rm_Api_.Service_RM_API_Init(65, NULL);
    // tcp连接
    m_sockhand_ = Rm_Api_.Service_Arm_Socket_Start((char*)tcp_ip_, tcp_port_, 5000);
    RCLCPP_INFO(this->get_logger(), "m_sockhand_ = %d", m_sockhand_);
    if (m_sockhand_ == INIT_SOCKET_ERR)
    {
        RCLCPP_INFO(this->get_logger(), "Socket 初始化失败");
    }
    else if (m_sockhand_ == SOCKET_CONNECT_ERR)
    {
        RCLCPP_INFO(this->get_logger(), "Socket 连接失败");
    }
    else if (m_sockhand_ == SOCKET_SEND_ERR)
    {
        RCLCPP_INFO(this->get_logger(), "Socket 发送失败");
    }
    else if (m_sockhand_ == SOCKET_TIME_OUT)
    {
        RCLCPP_INFO(this->get_logger(), "Socket 通讯超时");
    }
    else
    {
        // 设置通讯端口为 ModbusRTU 模式
        res = Rm_Api_.Service_Set_Modbus_Mode(m_sockhand_, rm_port_, dh_baudrate_, 5000, block_);
        if (0 != res )
        {
            RCLCPP_INFO(this->get_logger(), "设置通讯端口为 ModbusRTU 模式失败, res = %d", res);
        }
        // 初始化夹爪
        res = Rm_Api_.Service_Write_Single_Register(m_sockhand_, rm_port_, dh_init_addr_, 1, dh_addr_, block_);
        if (0 != res )
        {
            RCLCPP_INFO(this->get_logger(), "夹爪初始化失败, res = %d", res);
        }
        sleep(3);
        // 设置力矩
        res = Rm_Api_.Service_Write_Single_Register(m_sockhand_, rm_port_, dh_effort_addr_, 20, dh_addr_, block_);
        if (0 != res )
        {
            RCLCPP_INFO(this->get_logger(), "设置力矩失败, res = %d", res);
        }
        // 设置速度
        res = Rm_Api_.Service_Write_Single_Register(m_sockhand_, rm_port_, dh_speed_addr_, dh_speed_, dh_addr_, block_);
        if (0 != res )
        {
            RCLCPP_INFO(this->get_logger(), "设置速度失败, res = %d", res);
        }

        // 读取关节信息
        res = Rm_Api_.Service_Get_Joint_Degree(m_sockhand_, joint_);
        if (0 != res )
        {
            RCLCPP_INFO(this->get_logger(), "读取关节信息失败, res = %d", res);
        }
        // 读取当前夹爪位置
        res = Rm_Api_.Service_Get_Read_Input_Registers(m_sockhand_, rm_port_, dh_position_feedback_addr_, dh_addr_, &coils_data_);
        if (0 != res )
        {
            RCLCPP_INFO(this->get_logger(), "读取夹爪位置失败, res = %d", res);
        }

        RCLCPP_INFO(this->get_logger(), "初始化完成");
        //获取当前工作坐标系
        res = Rm_Api_.Service_Get_Current_Work_Frame(m_sockhand_, &frame_);
        joint_state_msg_.header.frame_id = frame_.frame_name.name;
        joint_action_state_msg_.header.frame_id = frame_.frame_name.name;
        //启动joint_state线程
        // joint_state_thread_ = std::thread(&Dh_Gripper::publish_joint_states_thread, this);

        res = Rm_Api_.Service_Change_Work_Frame(m_sockhand_, "Base", RM_BLOCK);
    }
}

int Dh_Gripper::MovePose(Pose pose, float *joint)
{
    u_int32_t res = 0;

    float joint_read_current[7];
    res = Rm_Api_.Service_Algo_Inverse_Kinematics(joint, &pose, joint_read_current, 1);

    if(0 != res)
    {
        RCLCPP_ERROR(this->get_logger(), "jie suan shi bai");
        return res;
    }

    for(int i = 0; i < 7; ++i)
    {
        joint[i] = joint_read_current[i];
    }
    // aim status
    publish_action_joint_states_thread(pose, joint_read_current);
    // current status
    publish_joint_states_thread();
    res = Rm_Api_.Service_Movej_CANFD(m_sockhand_, joint_read_current, 0, 0.0);

    return res;
}

bool Dh_Gripper::get_pico_msgs(const std_msgs::msg::String::SharedPtr msg, pico_msgs& msgs)
{

    // deprese Json data
    Json::Value root;
    Json::Reader reader;
    bool parsingSuccessful = reader.parse(msg->data, root);

    if(!parsingSuccessful)
    {
        RCLCPP_ERROR(this->get_logger(), "failed to parse JSON: %s", reader.getFormattedErrorMessages().c_str());
        return false;
    }
    
    for(size_t i = 0; i < root.size(); ++i)
    {
        // now, just right hand
        if(static_cast<int>(i) != 1)
        {
            continue;
        }
        const Json::Value& obj = root[static_cast<int>(i)];
        // position
        msgs.x = obj["position"]["x"].asDouble();
        msgs.y = obj["position"]["y"].asDouble();
        msgs.z = obj["position"]["z"].asDouble();
        // rotation
        msgs.w = obj["rotation"]["w"].asDouble();
        msgs.rx = obj["rotation"]["x"].asDouble();
        msgs.ry = obj["rotation"]["y"].asDouble();
        msgs.rz = obj["rotation"]["z"].asDouble();
        // other
        msgs.axisClick = obj["axisClick"].asString();
        msgs.axisX = obj["axisX"].asDouble();
        msgs.axisY = obj["axisY"].asDouble();
        msgs.indexTrig = obj["indexTrig"].asDouble();
        msgs.handTrig = obj["handTrig"].asBool();
        msgs.keyOne = obj["keyOne"].asString();
        msgs.keyTwo = obj["keyTwo"].asString();
        msgs.ts = obj["ts"].asInt64();

        RCLCPP_INFO(this->get_logger(), "Group : %zu:", i);
        RCLCPP_INFO(this->get_logger(), "Position : x=%.17f, y=%.17f, z=%.17f", msgs.x, msgs.y, msgs.z);
        RCLCPP_INFO(this->get_logger(), "Position : w=%.17f, x=%.17f, y=%.17f, z=%.17f", msgs.w, msgs.rx, msgs.ry, msgs.rz);
        RCLCPP_INFO(this->get_logger(), "axisClick : %s", msgs.axisClick.c_str());
        RCLCPP_INFO(this->get_logger(), "axisX : %.17f, axisY : %.17f", msgs.axisX, msgs.axisY);
        RCLCPP_INFO(this->get_logger(), "indexTrig : %.17f", msgs.indexTrig);
        RCLCPP_INFO(this->get_logger(), "handTrig : %s", msgs.handTrig ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "keyone : %s, keytwo : %s", msgs.keyOne.c_str(), msgs.keyTwo.c_str());
        RCLCPP_INFO(this->get_logger(), "timestamp : %lld", msgs.ts);
    }

    return true;
}

int call_count = 0;
/* 收到joy后调用的回调函数 */
void Dh_Gripper::joyCallback(const std_msgs::msg::String::SharedPtr msg)
{
    
    if(call_count % 10  != 0)
    {
        // return;
    }
    call_count++;

    u_int32_t res = 0;

    std::cout << "is in joyCallback" <<std::endl;
    pico_msgs msgs;
    if(!get_pico_msgs(msg, msgs))
    {
        return;
    }

    // gripper
    if (msgs.indexTrig == 0.0)
    {
        coils_data_ += dh_Step_;
        if (coils_data_ > 1000) coils_data_ = 1000;
        // 向外打开 dh_Step_
        res = Rm_Api_.Service_Write_Single_Register(m_sockhand_, rm_port_, dh_position_addr_, coils_data_, dh_addr_, block_);
        if (0 == res)
        {
            RCLCPP_INFO(this->get_logger(), "open failed");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "open failed");
        }
    }
    else if (msgs.indexTrig == 1.0)
    {
        coils_data_ -= dh_Step_;
        if (coils_data_ < 0) coils_data_ = 0;
        // 向内闭合 dh_Step_
        res = Rm_Api_.Service_Write_Single_Register(m_sockhand_, rm_port_, dh_position_addr_, coils_data_, dh_addr_, block_);
        if (0 == res)
        {
            RCLCPP_INFO(this->get_logger(), "close failed");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "close failed");
        }
    }
    // hand
    if(!msgs.handTrig)
    {
        last_state = false;
        return;
    }
    if(!last_state)
    {
        last_state = true;
        res = Rm_Api_.Service_Get_Current_Arm_State(m_sockhand_, joint_read_, &init_pose_, &Arm_Err, &Sys_Err);
        if (0 != res)
        {
            return;
        }
        else
        {
            Quat quat_ = Rm_Api_.Service_Algo_Euler2Quaternion(init_pose_.euler);//将欧拉角转为四元数
            init_pose_.quaternion.x = quat_.x;
            init_pose_.quaternion.y = quat_.y;
            init_pose_.quaternion.z = quat_.z;
            init_pose_.quaternion.w = quat_.w;
        }
    }
    Pose current_pose(init_pose_);
    current_pose.position.x = current_pose.position.x - msgs.x / 2.5;
    current_pose.position.y = current_pose.position.y - msgs.z / 2.5;
    current_pose.position.z = current_pose.position.z + msgs.y / 2.5;
    // current_pose.quaternion.z = current_pose.quaternion.z + msgs.rz;
    // {
    //     Euler euler = Rm_Api_.Service_Algo_Quaternion2Euler(init_pose_.quaternion);
    //     current_pose.euler.rx = euler.rx;
    //     current_pose.euler.ry = euler.ry;
    //     current_pose.euler.rz = euler.rz;
    // }
    res = MovePose(current_pose, joint_read_);
    if (0 == res)
    {
        RCLCPP_INFO(this->get_logger(), "******** Successfully ********");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "******** Failed ********");
    }
}

void Dh_Gripper::publish_action_joint_states_thread(Pose pose, float *joint)
{
    // 获取当前时间
    auto now = this->now();
    // 更新 joint_state_msg_ 的时间戳
    joint_action_state_msg_.header.stamp = now;

    // 读取机械臂joint的数据
    joint_action_state_msg_.position[0] = joint[0];
    joint_action_state_msg_.position[1] = joint[1];
    joint_action_state_msg_.position[2] = joint[2];
    joint_action_state_msg_.position[3] = joint[3];
    joint_action_state_msg_.position[4] = joint[4];
    joint_action_state_msg_.position[5] = joint[5];
    joint_action_state_msg_.position[6] = coils_data_; // 将手柄的position数据赋值给msg
    // 读取机械臂的位置（以米为单位)
    action_pose_msg_.position.x = pose_.position.x;
    action_pose_msg_.position.y = pose_.position.y;
    action_pose_msg_.position.z = pose_.position.z;
    // 设置机械臂的姿态（以欧拉角表示）
    action_pose_msg_.orientation.x = pose_.euler.rx;
    action_pose_msg_.orientation.y = pose_.euler.ry;
    action_pose_msg_.orientation.z = pose_.euler.rz;
    
    u_int32_t res;
    res = Rm_Api_.Service_Get_Joint_Speed(m_sockhand_, speed_);
    if(0 == res)
    {
        // 读取机械臂关节速度的数据
        joint_action_state_msg_.velocity[0] = speed_[0];
        joint_action_state_msg_.velocity[1] = speed_[1];
        joint_action_state_msg_.velocity[2] = speed_[2];
        joint_action_state_msg_.velocity[3] = speed_[3];
        joint_action_state_msg_.velocity[4] = speed_[4];
        joint_action_state_msg_.velocity[5] = speed_[5];
        joint_action_state_msg_.velocity[6] = dh_speed_;
    }

    //  发布 joint_state_msg_
    joint_action_state_publisher_->publish(joint_action_state_msg_);
    // 发布 pose_msg_
    action_publisher_->publish(action_pose_msg_);
    // std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 休眠20毫秒
}

// 发布机械臂和夹爪的joint_state
void Dh_Gripper::publish_joint_states_thread()
{
    // 获取当前时间
    auto now = this->now();
    // 更新 joint_state_msg_ 的时间戳
    joint_state_msg_.header.stamp = now;

    u_int32_t res;
    res = Rm_Api_.Service_Get_Current_Arm_State(m_sockhand_, joint_read_, &pose_, &Arm_Err, &Sys_Err);
    if (0 == res)
    {
        // 读取机械臂joint的数据
        joint_state_msg_.position[0] = joint_read_[0];
        joint_state_msg_.position[1] = joint_read_[1];
        joint_state_msg_.position[2] = joint_read_[2];
        joint_state_msg_.position[3] = joint_read_[3];
        joint_state_msg_.position[4] = joint_read_[4];
        joint_state_msg_.position[5] = joint_read_[5];
        joint_state_msg_.position[6] = coils_data_; // 将手柄的position数据赋值给msg
        // 读取机械臂的位置（以米为单位)
        pose_msg_.position.x = pose_.position.x;
        pose_msg_.position.y = pose_.position.y;
        pose_msg_.position.z = pose_.position.z;

        // 设置机械臂的姿态（以欧拉角表示）
        pose_msg_.orientation.x = pose_.euler.rx;
        pose_msg_.orientation.y = pose_.euler.ry;
        pose_msg_.orientation.z = pose_.euler.rz;
        // 设置机械臂的姿态（以四元数表示）
        //quat_ = Rm_Api_.Service_Algo_Euler2Quaternion(pose_.euler);//将欧拉角转为四元数（若想以四元数表示，可放开注释）
        //赋值四元数（若想以四元数表示，可放开注释）
        // pose_msg_.orientation.x = quat_.x;
        // pose_msg_.orientation.y = quat_.y;
        // pose_msg_.orientation.z = quat_.z;
        // pose_msg_.orientation.w = quat_.w;

    }
    res = Rm_Api_.Service_Get_Joint_Speed(m_sockhand_, speed_);
    if(0 == res)
    {
        // 读取机械臂关节速度的数据
        joint_state_msg_.velocity[0] = speed_[0];
        joint_state_msg_.velocity[1] = speed_[1];
        joint_state_msg_.velocity[2] = speed_[2];
        joint_state_msg_.velocity[3] = speed_[3];
        joint_state_msg_.velocity[4] = speed_[4];
        joint_state_msg_.velocity[5] = speed_[5];
        joint_state_msg_.velocity[6] = dh_speed_;
    }

    //  发布 joint_state_msg_
    joint_state_publisher_->publish(joint_state_msg_);
    // 发布 pose_msg_
    publisher_->publish(pose_msg_);
    // std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 休眠20毫秒
}

/* 主函数主要用于动作订阅和套接字通信 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Dh_Gripper>("rm_gripper_control"));
  rclcpp::shutdown();
  return 0;
}
