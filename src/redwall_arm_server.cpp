/* ROS action server */
#include <ros/ros.h>
#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>

/* 三次样条插补 */
#include <string.h>
#include <stddef.h>
#include <stdio.h>
#include "cubicSpline.h"

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
static const AmsNetId remoteNetId{192, 168, 0, 231, 1, 1};
static const char remoteIpV4[] = "114.213.237.49";

AdsDevice route{remoteIpV4, remoteNetId, AMSPORT_R0_PLC_TC3};

//action callback
AdsVariable<array<double, 1200>> pos_arr{route, "MAIN.pos_arr1"};
AdsVariable<bool> write_do_arr{route, "MAIN.write_do"};
AdsVariable<bool> read_do_arr{route, "MAIN.read_do"};
AdsVariable<bool> FIFO_write{route, "MAIN.FiFo_write_do"};
AdsVariable<bool> FIFO_overwrite{route, "MAIN.FiFo_overwrite_do"};
AdsVariable<bool> integrate_axis{route, "MAIN.integrate_do"};
AdsVariable<bool> disintegrate_axis{route, "MAIN.disintegrate_do"};
AdsVariable<bool> SetChannel_axis{route, "MAIN.SetChannel_do"};
AdsVariable<bool> start_fifo{route, "MAIN.start_do"};
AdsVariable<bool> stop_fifo{route, "MAIN.stop_do"};
AdsVariable<int> axis1_data_remain{route, "MAIN.FiFo_GetDimension.iNoOfFifoEntries"};

// move motors to achieve robot goals
AdsVariable<double> axis1_pos{route, "MAIN.pos_axis1"};
AdsVariable<double> axis2_pos{route, "MAIN.pos_axis2"};
AdsVariable<double> axis3_pos{route, "MAIN.pos_axis3"};
AdsVariable<double> axis4_pos{route, "MAIN.pos_axis4"};
AdsVariable<double> axis5_pos{route, "MAIN.pos_axis5"};
AdsVariable<double> axis6_pos{route, "MAIN.pos_axis6"};

// read angles for topic ---  /joint_states
AdsVariable<double> axis1_read_pos{route, "MAIN.pos_read_axis1"};
AdsVariable<double> axis2_read_pos{route, "MAIN.pos_read_axis2"};
AdsVariable<double> axis3_read_pos{route, "MAIN.pos_read_axis3"};
AdsVariable<double> axis4_read_pos{route, "MAIN.pos_read_axis4"};
AdsVariable<double> axis5_read_pos{route, "MAIN.pos_read_axis5"};
AdsVariable<double> axis6_read_pos{route, "MAIN.pos_read_axis6"};

// FiFo运行的位置点数目（单轴）
AdsVariable<unsigned int> fifo_rows{route, "MAIN.RowsNum"};

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


/* 客户端套接字文件描述符和地址,端口 */
typedef struct MySocketInfo{
    int socketCon;
    char *ipaddr;
    uint16_t port;
}_MySocketInfo;

/* 客户端连接所用数据的存储数组 */
struct MySocketInfo arrConSocket[10];
int conClientCount = 0;              // 连接的客户端个数

/* 客户端连接所用套接字的存储数组 */
pthread_t arrThrReceiveClient[10];
int thrReceiveClientCount = 0;       // 接受数据线程个数

/* action 服务端声明 */
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

/* 初始化输入输出速度加速度 */
double acc = 0, vel = 0;
double x_out = 0, y_out = 0;
/* 规划的路点数目 */
int point_num;
/* 判断路点数据是否改变 */
bool point_changed = false;

/* 三次样条无参构造 */
cubicSpline::cubicSpline()
{
}
/* 析构 */
cubicSpline::~cubicSpline()
{
    releaseMem();
}
/* 初始化参数 */
void cubicSpline::initParam()
{
    x_sample_ = y_sample_ = M_ = NULL;
    sample_count_ = 0;
    bound1_ = bound2_ = 0;
}
/* 释放参数 */
void cubicSpline::releaseMem()
{
    delete x_sample_;
    delete y_sample_;
    delete M_;

    initParam();
}
/* 加载关节位置数组等信息 */
bool cubicSpline::loadData(double *x_data, double *y_data, int count, double bound1, double bound2, BoundType type)
{
    if ((NULL == x_data) || (NULL == y_data) || (count < 3) || (type > BoundType_Second_Derivative) || (type < BoundType_First_Derivative))
    {
        return false;
    }

    initParam();

    x_sample_ = new double[count];
    y_sample_ = new double[count];
    M_        = new double[count];
    sample_count_ = count;

    memcpy(x_sample_, x_data, sample_count_*sizeof(double));
    memcpy(y_sample_, y_data, sample_count_*sizeof(double));

    bound1_ = bound1;
    bound2_ = bound2;

    return spline(type);
}
/* 计算样条插值 */
bool cubicSpline::spline(BoundType type)
{
    if ((type < BoundType_First_Derivative) || (type > BoundType_Second_Derivative))
    {
        return false;
    }

    //  追赶法解方程求二阶偏导数
    double f1=bound1_, f2=bound2_;

    double *a=new double[sample_count_];                //  a:稀疏矩阵最下边一串数
    double *b=new double[sample_count_];                //  b:稀疏矩阵最中间一串数
    double *c=new double[sample_count_];                //  c:稀疏矩阵最上边一串数
    double *d=new double[sample_count_];

    double *f=new double[sample_count_];

    double *bt=new double[sample_count_];
    double *gm=new double[sample_count_];

    double *h=new double[sample_count_];

    for(int i=0;i<sample_count_;i++)
        b[i]=2;                                //  中间一串数为2
    for(int i=0;i<sample_count_-1;i++)
        h[i]=x_sample_[i+1]-x_sample_[i];      // 各段步长
    for(int i=1;i<sample_count_-1;i++)
        a[i]=h[i-1]/(h[i-1]+h[i]);
    a[sample_count_-1]=1;

    c[0]=1;
    for(int i=1;i<sample_count_-1;i++)
        c[i]=h[i]/(h[i-1]+h[i]);

    for(int i=0;i<sample_count_-1;i++)
        f[i]=(y_sample_[i+1]-y_sample_[i])/(x_sample_[i+1]-x_sample_[i]);

    for(int i=1;i<sample_count_-1;i++)
        d[i]=6*(f[i]-f[i-1])/(h[i-1]+h[i]);

    //  追赶法求解方程
    if(BoundType_First_Derivative == type)
    {
        d[0]=6*(f[0]-f1)/h[0];
        d[sample_count_-1]=6*(f2-f[sample_count_-2])/h[sample_count_-2];

        bt[0]=c[0]/b[0];
        for(int i=1;i<sample_count_-1;i++)
            bt[i]=c[i]/(b[i]-a[i]*bt[i-1]);

        gm[0]=d[0]/b[0];
        for(int i=1;i<=sample_count_-1;i++)
            gm[i]=(d[i]-a[i]*gm[i-1])/(b[i]-a[i]*bt[i-1]);

        M_[sample_count_-1]=gm[sample_count_-1];
        for(int i=sample_count_-2;i>=0;i--)
            M_[i]=gm[i]-bt[i]*M_[i+1];
    }
    else if(BoundType_Second_Derivative == type)
    {
        d[1]=d[1]-a[1]*f1;
        d[sample_count_-2]=d[sample_count_-2]-c[sample_count_-2]*f2;

        bt[1]=c[1]/b[1];
        for(int i=2;i<sample_count_-2;i++)
            bt[i]=c[i]/(b[i]-a[i]*bt[i-1]);

        gm[1]=d[1]/b[1];
        for(int i=2;i<=sample_count_-2;i++)
            gm[i]=(d[i]-a[i]*gm[i-1])/(b[i]-a[i]*bt[i-1]);

        M_[sample_count_-2]=gm[sample_count_-2];
        for(int i=sample_count_-3;i>=1;i--)
            M_[i]=gm[i]-bt[i]*M_[i+1];

        M_[0]=f1;
        M_[sample_count_-1]=f2;
    }
    else
        return false;

    delete a;
    delete b;
    delete c;
    delete d;
    delete gm;
    delete bt;
    delete f;
    delete h;

    return true;
}
/* 得到速度和加速度数组 */
bool cubicSpline::getYbyX(double &x_in, double &y_out)
{
    int klo,khi,k;
    klo=0;
    khi=sample_count_-1;
    double hh,bb,aa;

    //  二分法查找x所在区间段
    while(khi-klo>1)
    {
        k=(khi+klo)>>1;
        if(x_sample_[k]>x_in)
            khi=k;
        else
            klo=k;
    }
    hh=x_sample_[khi]-x_sample_[klo];

    aa=(x_sample_[khi]-x_in)/hh;
    bb=(x_in-x_sample_[klo])/hh;

    y_out=aa*y_sample_[klo]+bb*y_sample_[khi]+((aa*aa*aa-aa)*M_[klo]+(bb*bb*bb-bb)*M_[khi])*hh*hh/6.0;

    //////test
    acc = (M_[klo]*(x_sample_[khi]-x_in) + M_[khi]*(x_in - x_sample_[klo])) / hh;
    vel = M_[khi]*(x_in - x_sample_[klo]) * (x_in - x_sample_[klo]) / (2 * hh)
          - M_[klo]*(x_sample_[khi]-x_in) * (x_sample_[khi]-x_in) / (2 * hh)
          + (y_sample_[khi] - y_sample_[klo])/hh
          - hh*(M_[khi] - M_[klo])/6;
    //printf("[---位置、速度、加速度---]");
    //printf("%0.9f, %0.9f, %0.9f\n",y_out, vel, acc);
    //////test end

    return true;
}

void trackMove(array<double, 1200>& arr){
    // AdsVariable<array<double, 1200>> pos_arr{route, "MAIN.pos_arr1"};
    // AdsVariable<bool> write_do_arr{route, "MAIN.write_do"};
    // AdsVariable<bool> read_do_arr{route, "MAIN.read_do"};
    // AdsVariable<bool> FIFO_write{route, "MAIN.FiFo_write_do"};
    // AdsVariable<bool> FIFO_overwrite{route, "MAIN.FiFo_overwrite_do"};
    // AdsVariable<bool> integrate_axis{route, "MAIN.integrate_do"};
    // AdsVariable<bool> disintegrate_axis{route, "MAIN.disintegrate_do"};
    // AdsVariable<bool> SetChannel_axis{route, "MAIN.SetChannel_do"};
    // AdsVariable<bool> start_fifo{route, "MAIN.start_do"};
    // AdsVariable<bool> stop_fifo{route, "MAIN.stop_do"};
    // AdsVariable<int> axis1_data_remain{route, "MAIN.FiFo_GetDimension.iNoOfFifoEntries"};
    mtx.lock();

    // init param
    stop_fifo = false;
    disintegrate_axis = true;
    write_do_arr = false;
    read_do_arr = false;
    FIFO_overwrite = false;
    FIFO_write = false;
    integrate_axis = false;
    SetChannel_axis = false;
    disintegrate_axis = false;
    start_fifo = false;
    ros::Duration(0.1).sleep();

    pos_arr = arr;
    ROS_INFO("Write xml and Read xml!");
    write_do_arr = true;
    ros::Duration(0.1).sleep();
    read_do_arr = true;
    ros::Duration(0.1).sleep();
    if(axis1_data_remain == 0 || First_Pos_Change)
    {
        ROS_INFO("fifo empty, write fifo data");
        FIFO_write = true;
    }
    else{
        ROS_INFO("overwrite fifo data");
        FIFO_overwrite = true;
    }
    ros::Duration(0.2).sleep();
    integrate_axis = true;
    SetChannel_axis = true;
    ros::Duration(0.5).sleep();
    start_fifo = true;
    ros::Duration(3.0).sleep();
    stop_fifo = true;
    disintegrate_axis = true;
    write_do_arr = false;
    read_do_arr = false;
    FIFO_overwrite = false;
    FIFO_write = false;
    integrate_axis = false;
    SetChannel_axis = false;
    disintegrate_axis = false;
    start_fifo = false;

    mtx.unlock();
}

/* 收到action的goal后调用的回调函数 */
void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as) {

    array<double, 1200> pos_tras;
    //pos_tras.clear();
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
        // string joint1 = goal->trajectory.joint_names[0];
        // cout<<"joint1: " <<joint1<<endl;
        // string joint2 = goal->trajectory.joint_names[1];
        // cout<<"joint2: " <<joint2<<endl;
        // string joint3 = goal->trajectory.joint_names[2];
        // cout<<"joint3: " <<joint3<<endl;
        // string joint4 = goal->trajectory.joint_names[3];
        // cout<<"joint4: " <<joint4<<endl;
        // string joint5 = goal->trajectory.joint_names[4];
        // cout<<"joint5: " <<joint5<<endl;
        // string joint6 = goal->trajectory.joint_names[5];
        // cout<<"joint6: " <<joint6<<endl;

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
    cout<<p_lumbar[point_num-1]<<endl;
    cout<<p_big_arm[point_num-1]<<endl;
    cout<<p_small_arm[point_num-1]<<endl;
    cout<<p_wrist[point_num-1]<<endl;
    cout<<p_hand[point_num-1]<<endl;
    cout<<p_finger[point_num-1]<<endl;

    // 实例化样条
    cubicSpline spline;

    //索引错误，array对应的减速比与对应的joint都要重新映射
    // lumbar
    spline.loadData(time_from_start, p_lumbar, point_num, 0, 0, cubicSpline::BoundType_First_Derivative);
    time_from_start_.clear();
    p_lumbar_.clear();
    v_lumbar_.clear();
    a_lumbar_.clear();
    x_out = 0;
    double rate = (time_from_start[point_num-1] - time_from_start[0])/fifo_data;
    for (int k = 0; k <= fifo_data; k++) {
        spline.getYbyX(x_out, y_out);
        time_from_start_.push_back(x_out);
        p_lumbar_.push_back(y_out);
        v_lumbar_.push_back(vel);
        a_lumbar_.push_back(acc);
        pos_tras[k * 6 + 1] = y_out / M_PI * 180 * joint_radio[1];
        x_out += rate;
    }
    cout<<p_lumbar_.back()<<endl;
    // big_arm
    spline.loadData(time_from_start, p_big_arm, point_num, 0, 0, cubicSpline::BoundType_First_Derivative);
    p_big_arm_.clear();
    v_big_arm_.clear();
    a_big_arm_.clear();
    x_out = 0;
    for (int k = 0; k <= fifo_data ; k++) {
        spline.getYbyX(x_out, y_out);
        p_big_arm_.push_back(y_out);
        v_big_arm_.push_back(vel);
        a_big_arm_.push_back(acc);
        pos_tras[k * 6 + 5] = y_out / M_PI * 180 * joint_radio[5];
        x_out += rate;
    }
    cout<<p_big_arm_.back()<<endl;
    // small_arm
    spline.loadData(time_from_start, p_small_arm, point_num, 0, 0, cubicSpline::BoundType_First_Derivative);
    p_small_arm_.clear();
    v_small_arm_.clear();
    a_small_arm_.clear();
    x_out = 0;
    for (int k = 0; k <= fifo_data ; k++) {
        spline.getYbyX(x_out, y_out);
        p_small_arm_.push_back(y_out);
        v_small_arm_.push_back(vel);
        a_small_arm_.push_back(acc);
        pos_tras[k * 6 + 4] = y_out / M_PI * 180 * joint_radio[4];
        x_out += rate;
    }
    cout<<p_small_arm_.back()<<endl;
    // wrist
    spline.loadData(time_from_start, p_wrist, point_num, 0, 0, cubicSpline::BoundType_First_Derivative);
    p_wrist_.clear();
    v_wrist_.clear();
    a_wrist_.clear();
    x_out = 0;
    for (int k = 0; k <= fifo_data; k++) {
        spline.getYbyX(x_out, y_out);
        p_wrist_.push_back(y_out);
        v_wrist_.push_back(vel);
        a_wrist_.push_back(acc);
        pos_tras[k * 6] = y_out / M_PI * 180 * joint_radio[0];
        x_out += rate;
    }
    cout<<p_wrist_.back()<<endl;
    // hand
    spline.loadData(time_from_start, p_hand, point_num, 0, 0, cubicSpline::BoundType_First_Derivative);
    p_hand_.clear();
    v_hand_.clear();
    a_hand_.clear();
    x_out = 0;
    for (int k = 0; k <= fifo_data; k++) {
        spline.getYbyX(x_out, y_out);
        p_hand_.push_back(y_out);
        v_hand_.push_back(vel);
        a_hand_.push_back(acc);
        pos_tras[k * 6 + 2] = y_out / M_PI * 180 * joint_radio[2];
        x_out += rate;
    }
    cout<<p_hand_.back()<<endl;

    // finger
    spline.loadData(time_from_start, p_finger, point_num, 0, 0, cubicSpline::BoundType_First_Derivative);
    p_finger_.clear();
    v_finger_.clear();
    a_finger_.clear();
    x_out = 0;
    for (int k = 0; k <= fifo_data; k++) {
        spline.getYbyX(x_out, y_out);
        p_finger_.push_back(y_out);
        v_finger_.push_back(vel);
        a_finger_.push_back(acc);
        pos_tras[k * 6 + 3] = y_out / M_PI * 180 * joint_radio[3];
        x_out += rate;
    }
    cout<<p_finger_.back()<<endl;
    cout<<"arrray output"<<endl;
    for(int k = 1194; k<1200; k++)
    {
        cout<<pos_tras[k]<<endl;
    }
    //control_msgs::FollowJointTrajectoryFeedback feedback;
    //feedback = NULL;
    //as->publishFeedback(feedback);
    /* move_group规划的路径包含的路点个数 */
    int point_num_s = p_finger_.size();
    ROS_INFO("Finally Move_group give us %d points",point_num_s);
    ROS_INFO("Now We get all joints P,V,A,T!");
    point_changed = true;
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
    ros::init(argc, argv, "redwall_arm_control");
    ros::NodeHandle nh;

    joint_radio.resize(6);
    joint_radio[0] = 45;
    joint_radio[1] = 100;
    joint_radio[2] = 100; 
    joint_radio[3] = 50;
    joint_radio[4] = 45;
    joint_radio[5] = 45;

    

    // AdsVariable<bool> variable_to_write{route, "MAIN.read_do"};
    // AdsVariable<array<double, 1000>> pos_arr{route, "MAIN.pos_arr1"};
    // array<double, 1000> arr;
    // for(int i = 0; i<1000; i++)
    // {
    //     arr[i] = 10 + 0.15 * i / M_PI;
    // }
    // pos_arr = arr;

    // variable_to_write = false;

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
