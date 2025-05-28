#include "rm_dh.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>

using namespace std;


/* 构造函数 */
Dh_Gripper::Dh_Gripper(std::string name) : Node(name)
{
    using namespace std::placeholders;
    u_int32_t res;

    // 创建订阅者，自动注册到节点
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy",                                                             // 话题名称
        1,                                                                 // 队列大小
        std::bind(&Dh_Gripper::joyCallback, this, std::placeholders::_1)); // 设置回调函数
    // 创建一个发布者，发布到"rm_dh_joint_states"话题，队列大小为10
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("rm_dh_joint_states", 10);
    joint_state_msg_.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "finger1_joint"};
    joint_state_msg_.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 设置关节位置
    joint_state_msg_.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 设置关节速度

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
        //启动joint_state线程
        joint_state_thread_ = std::thread(&Dh_Gripper::publish_joint_states_thread, this);

        res = Rm_Api_.Service_Change_Work_Frame(m_sockhand_, "Base", RM_BLOCK);
    }
}

int Dh_Gripper::MovePose(Pose pose, float *joint)
{
    u_int32_t res = 0;

    // res = Rm_Api_.Service_Movej_P_Cmd(m_sockhand_, pose, 20, 0, 0, RM_BLOCK);

    float joint_read_current[7];
    auto start = std::chrono::high_resolution_clock::now();
    res = Rm_Api_.Service_Algo_Inverse_Kinematics(joint, &pose, joint_read_current, 1);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    if(0 != res)
    {
        RCLCPP_ERROR(this->get_logger(), "jie suan shi bai");
        return res;
    }
    std::cout << "Service_Algo_Inverse_Kinematics time is : " << duration <<std::endl;

    start = std::chrono::high_resolution_clock::now();
    res = Rm_Api_.Service_Movej_CANFD(m_sockhand_, joint_read_current, 0, 0.0);
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::cout << "Service_Movej_CANFD time is : " << duration <<std::endl;

    return res;
}

/* 收到joy后调用的回调函数 */
void Dh_Gripper::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    u_int32_t res = 0;
    
    if (msg->buttons.size() > 0 || msg->axes.size() > 0)
    {
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

        // 检查back按钮是否被按下
        if (msg->buttons[8] == 1)
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
        // 检查start按钮是否被按下
        if (msg->buttons[9] == 1)
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

        // 关节1控制
        if (msg->axes[0] == 1)  //如果左侧方向键-上-被按下
        {
            init_pose_.position.y = init_pose_.position.y + 0.1;
            res = MovePose(init_pose_, joint_read_);
            // res = MoveJoint(rm_Step_, 0);
            if (0 == res)
            {
                RCLCPP_INFO(this->get_logger(), "方向键-上 is pressed. Moving joint_1 left:%f", init_pose_.position.y);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "方向键-上 is pressed. Moving joint_1 left:%f MoveJoint failed! res=%d", init_pose_.position.y, res);
            }
        }
        if (msg->axes[0] == -1)  //如果左侧方向键-下-被按下
        {
            init_pose_.position.y = init_pose_.position.y - 0.1;
            res = MovePose(init_pose_, joint_read_);
            // res = MoveJoint(-rm_Step_, 0);
            if (0 == res)
            {
                RCLCPP_INFO(this->get_logger(), "方向键-下 is pressed. Moving joint_1 righit:%f", init_pose_.position.y);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "方向键-下 is pressed. Moving joint_1 righit:%f MoveJoint failed! res=%d", init_pose_.position.y, res);
            }
        }
        // 关节2控制
        if (msg->axes[1] == 1)  //如果左侧方向键-左-被按下
        {
            init_pose_.position.x = init_pose_.position.x - 0.1;
            res = MovePose(init_pose_, joint_read_);
            // res = MoveJoint(-rm_Step_, 1);
            if (0 == res)
            {
                RCLCPP_INFO(this->get_logger(), "方向键-左 is pressed. Moving joint_2 left:%f", init_pose_.position.x);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "方向键-左 is pressed. Moving joint_2 left:%f MoveJoint failed! res=%d", init_pose_.position.x, res);
            }
        }
        if (msg->axes[1] == -1)  //如果左侧方向键-右-被按下
        {
            init_pose_.position.x = init_pose_.position.x + 0.1;
            res = MovePose(init_pose_, joint_read_);
            // res = MoveJoint(rm_Step_, 1);
            if (0 == res)
            {
                RCLCPP_INFO(this->get_logger(), "方向键-右 is pressed. Moving joint_2 righit:%f", init_pose_.position.x);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "方向键-右 is pressed. Moving joint_2 righit:%f MoveJoint failed! res=%d", init_pose_.position.x, res);
            }
        }
        // 关节3控制
        if (msg->buttons[4] == 1)  // 如果左侧LB键被按下
        {
            init_pose_.position.z = init_pose_.position.z + 0.1;
            res = MovePose(init_pose_, joint_read_);
            // res = MoveJoint(-rm_Step_, 2);
            if (0 == res)
            {
                RCLCPP_INFO(this->get_logger(), "左侧LB is pressed. Moving joint_3 left:%f", init_pose_.position.z);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "左侧LB is pressed. Moving joint_3 left:%f MoveJoint failed! res=%d", init_pose_.position.z, res);
            }
        }
        if (msg->buttons[6] == 1)  // 如果左侧LT键被按下
        {
            init_pose_.position.z = init_pose_.position.z - 0.1;
            res = MovePose(init_pose_, joint_read_);
            // res = MoveJoint(rm_Step_, 2);
            if (0 == res)
            {
                RCLCPP_INFO(this->get_logger(), "左侧LT is pressed. Moving joint_3 righit:%f", init_pose_.position.z);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "左侧LT is pressed. Moving joint_3 righit:%f MoveJoint failed! res=%d", init_pose_.position.z, res);
            }
        }
        // 关节4控制
        if (msg->buttons[2] == 1)  // 如果右侧功能键Y被按下
        {
            init_pose_.euler.ry = init_pose_.euler.ry + 0.1;
            res = MovePose(init_pose_, joint_read_);
            // res = MoveJoint(-rm_Step_, 3);
            if (0 == res)
            {
                RCLCPP_INFO(this->get_logger(), "功能键B is pressed. Moving joint_4 left:%f", init_pose_.euler.rx);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "功能键B is pressed. Moving joint_4 left:%f MoveJoint failed! res=%d", init_pose_.euler.rx, res);
            }
        }
        if (msg->buttons[0] == 1)  // 如果右侧功能键A被按下
        {
            init_pose_.euler.ry = init_pose_.euler.ry - 0.1;
            res = MovePose(init_pose_, joint_read_);
            // res = MoveJoint(rm_Step_, 3);
            if (0 == res)
            {
                RCLCPP_INFO(this->get_logger(), "功能键X is pressed. Moving joint_4 righit:%f", init_pose_.euler.rx);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "功能键X is pressed. Moving joint_4 righit:%f MoveJoint failed! res=%d", init_pose_.euler.rx, res);
            }
        }
         // 关节5控制
        if (msg->buttons[1] == 1)  // 如果右侧功能键X被按下
        {
            init_pose_.euler.rx = init_pose_.euler.rx - 0.1;
            res = MovePose(init_pose_, joint_read_);
            // res = MoveJoint(rm_Step_, 4);
            if (0 == res)
            {
                RCLCPP_INFO(this->get_logger(), "功能键A is pressed. Moving joint_5 left:%f", init_pose_.euler.ry);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "功能键A is pressed. Moving joint_5 left:%f MoveJoint failed! res=%d", init_pose_.euler.ry, res);
            }
        }
        if (msg->buttons[3] == 1)  // 如果右侧功能键B被按下
        {
            init_pose_.euler.rx = init_pose_.euler.rx + 0.1;
            res = MovePose(init_pose_, joint_read_);
            // res = MoveJoint(-rm_Step_, 4);
            if (0 == res)
            {
                RCLCPP_INFO(this->get_logger(), "功能键Y is pressed. Moving joint_5 left:%f", init_pose_.euler.ry);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "功能键Y is pressed. Moving joint_5 left:%f MoveJoint failed! res=%d", init_pose_.euler.ry, res);
            }
        }
        // 关节6控制
        if (msg->buttons[5] == 1)  // 如果右侧RB键被按下
        {
            init_pose_.euler.rz = init_pose_.euler.rz + 0.1;
            res = MovePose(init_pose_, joint_read_);
            // res = MoveJoint(-rm_Step_, 5);
            if (0 == res)
            {
                RCLCPP_INFO(this->get_logger(), "右侧RB is pressed. Moving joint_6 left:%f", init_pose_.euler.rz);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "右侧RB is pressed. Moving joint_6 left:%f MoveJoint failed! res=%d", init_pose_.euler.rz, res);
            }
        }
        if (msg->buttons[7] == 1)  // 如果右侧RT键被按下
        {
            init_pose_.euler.rz = init_pose_.euler.rz - 0.1;
            res = MovePose(init_pose_, joint_read_);
            // res = MoveJoint(rm_Step_, 5);
            if (0 == res)
            {
                RCLCPP_INFO(this->get_logger(), "右侧RT is pressed. Moving joint_6 left:%f", init_pose_.euler.rz);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "右侧RT is pressed. Moving joint_6 left:%f MoveJoint failed! res=%d", init_pose_.euler.rz, res);
            }
        }
    }
}

// 发布机械臂和夹爪的joint_state
void Dh_Gripper::publish_joint_states_thread()
{
    while (rclcpp::ok())
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
            // 设置机械臂的姿态（以四元数表示）
            //quat_ = Rm_Api_.Service_Algo_Euler2Quaternion(pose_.euler);//将欧拉角转为四元数（若想以四元数表示，可放开注释）

            // 设置机械臂的姿态（以欧拉角表示）
            pose_msg_.orientation.x = pose_.euler.rx;
            pose_msg_.orientation.y = pose_.euler.ry;
            pose_msg_.orientation.z = pose_.euler.rz;

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
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 休眠20毫秒
    }
}

/* 主函数主要用于动作订阅和套接字通信 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Dh_Gripper>("rm_gripper_control"));
  rclcpp::shutdown();
  return 0;
}
