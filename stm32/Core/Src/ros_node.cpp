#include "ros_node.h"
#include "rosserial_hardware.h"
#include "motor_control.h"

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <math.h>

/* 조향각(rad) 클램프 범위: 약 ±40도에 해당하는 ±0.7rad */
#define STEERING_RAD_LIMIT 0.7f
#define STEERING_CENTER_DEG 90.0f

static int32_t g_current_state = -1;   /* state_callback으로 갱신, 기본값 -1(초기) */
static volatile bool g_obstacle_near = false;

static ros::NodeHandle<STM32Hardware> nh;

static std_msgs::Int32 echo_msg;
static ros::Publisher echo_pub("stm32_echo", &echo_msg);

static void steering_callback(const std_msgs::Float32 &msg)
{
    float rad = msg.data;
    if (rad > STEERING_RAD_LIMIT)  rad = STEERING_RAD_LIMIT;
    if (rad < -STEERING_RAD_LIMIT) rad = -STEERING_RAD_LIMIT;

    /* rad -> deg 변환 후 중앙(90deg) 기준으로 보정값을 더해 서보 각도 산출 */
    float deg = STEERING_CENTER_DEG + rad * (180.0f / (float)M_PI);
    Servo_SetAngle(deg);
}

static void obstacle_callback(const std_msgs::Float32 &msg)
{
    /* LiDAR 최단 거리가 0.5m 이하면 장애물 근접으로 판단, 주행 차단 */
    g_obstacle_near = (msg.data <= 0.5f);
}

static void state_callback(const std_msgs::Int32 &msg)
{
    /* 0: 정지, 1: 수동 모드(현재는 정지 처리), 2: 자동 모드 */
    g_current_state = msg.data;
}

static ros::Subscriber<std_msgs::Float32> steering_sub("/steering_angle", &steering_callback);
static ros::Subscriber<std_msgs::Int32>   state_sub("/state_msg", &state_callback);
static ros::Subscriber<std_msgs::Float32> lidar_sub("/lidar_distance", &obstacle_callback);

extern "C" void ROS_Init(void)
{
    nh.initNode();
    nh.subscribe(steering_sub);
    nh.subscribe(state_sub);
    nh.subscribe(lidar_sub);
    nh.advertise(echo_pub);
}

extern "C" void ROS_SpinOnce(void)
{
    nh.spinOnce();
}

extern "C" int32_t ROS_GetState(void)
{
    return g_current_state;
}

extern "C" int ROS_ObstacleNear(void)
{
    return g_obstacle_near ? 1 : 0;
}

extern "C" void ROS_PublishEcho(int32_t value)
{
    echo_msg.data = value;
    echo_pub.publish(&echo_msg);
}
