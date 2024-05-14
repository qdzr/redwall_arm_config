/* ROS action server local ip: 169.254.210.10*/
#include <ros/ros.h>
#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>

/* 使用变长数组 */
#include <vector>
#include <algorithm>

/* 套接字通信 */
#include <sstream>
#include <stdlib.h>
#include <sys/types.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <signal.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <mutex>

/* connected Robot*/
#include "AdsLib.h"
#include "AdsNotification.h"
#include "AdsVariable.h"

#define _USE_MATH_DEFINES
#include <math.h>

#define PORT 7788
#define fifo_data 199
using namespace std;
vector<double> time_from_start_;
vector<double> p_lumbar_;
vector<double> p_big_arm_;
vector<double> p_small_arm_;
vector<double> p_wrist_;
vector<double> p_hand_;
vector<double> p_finger_;
vector<double> v_lumbar_;
vector<double> v_big_arm_;
vector<double> v_small_arm_;
vector<double> v_wrist_;
vector<double> v_hand_;
vector<double> v_finger_;
vector<double> a_lumbar_;
vector<double> a_big_arm_;
vector<double> a_small_arm_;
vector<double> a_wrist_;
vector<double> a_hand_;
vector<double> a_finger_;

sensor_msgs::JointState j_s;
vector<int> joint_radio;
bool First_Pos_Change = true;
bool FiFo_sensor = false;
double rad = M_PI / 180;

//connected robot
static const AmsNetId remoteNetId{172, 18, 214, 11, 1, 1};  // 172, 18, 214, 11, 1, 1              192, 168, 0, 231, 1, 1
static const char remoteIpV4[] = "169.254.210.196";   // 169.254.210.196            169.254.210.100

AdsDevice route{remoteIpV4, remoteNetId, AMSPORT_R0_PLC_TC3};

//action callback
AdsVariable<array<double, 1200>> pos_arr{route, "MAIN.pos_arr2"};

// AdsVariable<bool> write_do_arr{route, "MAIN.write_do"};
// AdsVariable<bool> read_do_arr{route, "MAIN.read_do"};
AdsVariable<bool> FIFO_write{route, "MAIN.FiFo_write_do"};
AdsVariable<bool> FIFO_overwrite{route, "MAIN.FiFo_overwrite_do"};
AdsVariable<bool> integrate_axis{route, "MAIN.integrate_do"};
AdsVariable<bool> disintegrate_axis{route, "MAIN.disintegrate_do"};
AdsVariable<bool> SetChannel_axis{route, "MAIN.SetChannel_do"};
AdsVariable<bool> start_fifo{route, "MAIN.start_do"};
AdsVariable<bool> stop_fifo{route, "MAIN.stop_do"};
// AdsVariable<bool> start_fifo_init{route, "MAIN.start_FIFO_Moving"};
// AdsVariable<bool> FIFO_Home_init{route, "MAIN.FIFO_Home"};


AdsVariable<unsigned int> axis1_data_remain{route, "MAIN.axis1.NcToPlc.SafEntries"};

// move motors to achieve robot goals
AdsVariable<double> axis1_pos{route, "MAIN.pos_axis1"};
AdsVariable<double> axis2_pos{route, "MAIN.pos_axis2"};
AdsVariable<double> axis3_pos{route, "MAIN.pos_axis3"};
AdsVariable<double> axis4_pos{route, "MAIN.pos_axis4"};
AdsVariable<double> axis5_pos{route, "MAIN.pos_axis5"};
AdsVariable<double> axis6_pos{route, "MAIN.pos_axis6"};

// FiFo运行的位置点数目（单轴）
AdsVariable<int> fifo_rows{route, "MAIN.RowsNum"};

mutex mtx;
/* 存储的结构体 p2*/
struct vel_data
{
    double time_from_begin;
    double lumbar_pos;
    double big_arm_pos;
    double small_arm_pos;
    double wrist_pos;
    double hand_pos;
    double finger_pos;
    double lumbar_vel;
    double big_arm_vel;
    double small_arm_vel;
    double wrist_vel;
    double hand_vel;
    double finger_vel;
    double lumbar_acc;
    double big_arm_acc;
    double small_arm_acc;
    double wrist_acc;
    double hand_acc;
    double finger_acc;
};

/* 数据收发结构体 */
struct vel_data p2;
char writebuf[sizeof(p2)];

/* action 服务端声明 */
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

/* 初始化输入输出速度加速度 */
double acc = 0, vel = 0;
double x_out = 0, y_out = 0;
/* 规划的路点数目 */
unsigned int point_num;
/* 判断路点数据是否改变 */
bool point_changed = false;

void trackMove(array<double, 1200>& arr){
    // AdsVariable<array<double, 1200>> pos_arr{route, "MAIN.pos_arr1"};

    mtx.lock();

    fifo_rows = point_num;
    double time_moving = 0.2 * (point_num + 5);

    // init param
    stop_fifo = false;
    disintegrate_axis = true;
    // write_do_arr = false;
    // read_do_arr = false;
    FIFO_overwrite = false;
    FIFO_write = false;
    integrate_axis = false;
    SetChannel_axis = false;
    disintegrate_axis = false;
    start_fifo = false;
    ros::Duration(0.1).sleep();

    pos_arr = arr;
    ROS_INFO("Write xml and Read xml already! Try to init parameters");
    // write_do_arr = true;
    // ros::Duration(0.1).sleep();
    // read_do_arr = true;
    ros::Duration(0.1).sleep();
    if(axis1_data_remain == 0 || First_Pos_Change)
    {
        ROS_INFO("fifo empty, write fifo data");
        FIFO_write = true;
    }
    else{
        ROS_INFO("Remain FIFO data... Try to overwrite fifo data");
        FIFO_overwrite = true;
    }

    SetChannel_axis = true;
    ros::Duration(0.2).sleep();

    integrate_axis = true;
    ros::Duration(0.2).sleep();

    start_fifo = true;
    ros::Duration(time_moving).sleep();

    // stop_fifo = true;
    // write_do_arr = false;
    // read_do_arr = false;
    start_fifo = false;
    FIFO_overwrite = false;
    FIFO_write = false;
    integrate_axis = false;
    SetChannel_axis = false;
    // disintegrate_axis = false;
    disintegrate_axis = true;

    mtx.unlock();
}

/* 收到action的goal后调用的回调函数 */
void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as) {

    array<double, 1200> pos_tras;

    /* move_group规划的路径包含的路点个数 */
    point_num = goal->trajectory.points.size();
    
    ROS_INFO("First Move_group give us %d points",point_num);

    /* 各个关节位置 */
    double p_lumbar[point_num];
    double p_big_arm[point_num];
    double p_small_arm[point_num];
    double p_wrist[point_num];
    double p_hand[point_num];
    double p_finger[point_num];

    /* 各个关节速度 */
    double v_lumbar[point_num];
    double v_big_arm[point_num];
    double v_small_arm[point_num];
    double v_wrist[point_num];
    double v_hand[point_num];
    double v_finger[point_num];

    /* 各个关节加速度 */
    double a_lumbar[point_num];
    double a_big_arm[point_num];
    double a_small_arm[point_num];
    double a_wrist[point_num];
    double a_hand[point_num];
    double a_finger[point_num];

    /* 时间数组 */
    double time_from_start[point_num];

    for (int i = 0; i < point_num; i++) {
        /* need resort*/
        /* Real joint1: big_arm  joint2: finger  joint3: hand joint4: lumbar  joint5: small_arm joint6: wrist */
        p_lumbar[i] = goal->trajectory.points[i].positions[0];
        p_big_arm[i] = goal->trajectory.points[i].positions[1];
        p_small_arm[i] = goal->trajectory.points[i].positions[2];
        p_wrist[i] = goal->trajectory.points[i].positions[3];
        p_hand[i] = goal->trajectory.points[i].positions[4];
        p_finger[i] = goal->trajectory.points[i].positions[5];

        v_lumbar[i] = goal->trajectory.points[i].velocities[0];
        v_big_arm[i] = goal->trajectory.points[i].velocities[1];
        v_small_arm[i] = goal->trajectory.points[i].velocities[2];
        v_wrist[i] = goal->trajectory.points[i].velocities[3];
        v_hand[i] = goal->trajectory.points[i].velocities[4];
        v_finger[i] = goal->trajectory.points[i].velocities[5];

        a_lumbar[i] = goal->trajectory.points[i].accelerations[0];
        a_big_arm[i] = goal->trajectory.points[i].accelerations[1];
        a_small_arm[i] = goal->trajectory.points[i].accelerations[2];
        a_wrist[i] = goal->trajectory.points[i].accelerations[3];
        a_hand[i] = goal->trajectory.points[i].accelerations[4];
        a_finger[i] = goal->trajectory.points[i].accelerations[5];
        

        time_from_start[i] = goal->trajectory.points[i].time_from_start.toSec();
    }

    //索引错误，array对应的减速比与对应的joint都要重新映射

    for(int k = 0; k < point_num; k++){
        // 映射lumbar   输出的p_lumbar对应的是big_arm的值
        pos_tras[k * 6 + 1] = p_lumbar[k] / M_PI * 180 * joint_radio[1];

        // 映射big_arm 输出的p_big_arm对应的是finger的值
        pos_tras[k * 6 + 5] = p_big_arm[k] / M_PI * 180 * joint_radio[5];

        // 映射small_arm 输出的p_small_arm对应的是hand的值
        pos_tras[k * 6 + 4] = p_small_arm[k] / M_PI * 180 * joint_radio[4];

        // 映射wrist  输出的wrist对应的是lumbar的值
        pos_tras[k * 6] = p_wrist[k] / M_PI * 180 * joint_radio[0];

        // 映射hand 输出的hand对应的是small_arm的值
        pos_tras[k * 6 + 2] = p_hand[k] / M_PI * 180 * joint_radio[2];

        // 映射finger 输出的finger对应的是wrist的值
        pos_tras[k * 6 + 3] = p_finger[k] / M_PI * 180 * joint_radio[3];
    }
    // 将剩余的数据初始化为0
    for(int k = point_num * 6; k<1200; k++){
        pos_tras[k] = 0;
    }
    for(int k = (point_num - 1) * 6; k < point_num * 6; k++){
        printf("末端电机角度： ");
        cout << pos_tras[k] << endl;
    }
    //control_msgs::FollowJointTrajectoryFeedback feedback;
    //feedback = NULL;
    //as->publishFeedback(feedback);
    /* move_group规划的路径包含的路点个数 */
    ROS_INFO("Now We get all joints P,V,A,T!");
    ROS_INFO("TwinCAT FIFO Ready Moving Motors!");
    trackMove(pos_tras);
    as->setSucceeded();
}

void UpdateJointStates(ros::Publisher& joint_pub, ros::Rate& loop_rate){
    j_s.header.stamp = ros::Time::now();
    j_s.name.resize(6);
    j_s.position.resize(6);
    j_s.name[0] = "lumbar";
    j_s.name[1] = "big_arm";
    j_s.name[2] = "small_arm";
    j_s.name[3] = "wrist";
    j_s.name[4] = "hand";
    j_s.name[5] = "finger";
    
    mtx.lock();

    j_s.position[0] = axis1_pos / joint_radio[0] * rad;
    j_s.position[1] = axis2_pos / joint_radio[1] * rad;
    j_s.position[2] = axis3_pos / joint_radio[2] * rad;
    j_s.position[3] = axis4_pos / joint_radio[3] * rad;
    j_s.position[4] = axis5_pos / joint_radio[4] * rad;
    j_s.position[5] = axis6_pos / joint_radio[5] * rad;
    mtx.unlock();
    // cout<<typeid(j_s.position).name()<<endl;
    joint_pub.publish(j_s);
    loop_rate.sleep();
}

/* 主函数主要用于动作订阅和套接字通信 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "redwall_arm_control_FIFO");
    ros::NodeHandle nh;

    joint_radio.resize(6);
    joint_radio[0] = 45;
    joint_radio[1] = 50;
    joint_radio[2] = 100; 
    joint_radio[3] = -50;
    joint_radio[4] = -70;   // 估计
    joint_radio[5] = -50;    // 估计

    // 定义一个服务器
    Server server(nh, "redwall/follow_joint_trajectory", boost::bind(&execute, _1, &server), false);
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ros::Rate loop_rate(10); //10Hz
    
    // 服务器开始运行
    server.start();
    //ros::spin();
    ROS_INFO("Server start!");

    while(ros::ok())
    {
        // printf("test!\n");
        UpdateJointStates(joint_pub, loop_rate);
        ros::spinOnce();
    }
    return 0;
}
