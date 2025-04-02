#ifndef GIMBAL_SERIAL_HPP_
#define GIMBAL_SERIAL_HPP_

#include <ros/ros.h>
#include <serial/serial.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>


#include "sentry_navigation/refree.h"

struct gimbal_send_msg{
    uint8_t header;
    float v_x;
    float v_y;
    float w_z;
    uint64_t config;
    float pose_x;
    float pose_y;
    float pose_z;
    float yaw;
    uint8_t tailer;
}__attribute__((packed));

class sentrySerial
{
public:
    sentrySerial();
    ~sentrySerial();

private:
    void openSerial(serial::Serial &serial, std::string &serial_port, uint32_t &serial_baudrate);
    void cmdVelCallback(geometry_msgs::TwistConstPtr msg);

public:
    ros::NodeHandle nh_;
    ros::NodeHandle pr_nh_;
    ros::Subscriber sub_vel_;
    ros::Publisher pub_goal_;
    ros::Publisher pub_refree_;

    geometry_msgs::Twist cmd_vel_;
    geometry_msgs::PoseStamped goal_;

    serial::Serial gimbal_serial_;
    std::string serial_port_;
    uint32_t serial_baudrate_;
    gimbal_send_msg* gimbal_send_;

    sentry_navigation::refree refree_msg_;

    unsigned char DATA_TX_BUFF[sizeof(gimbal_send_msg)];
    unsigned char DATA_RX_BUFF[sizeof(sentry_navigation::refree)];

    const uint8_t FRAME_REFREE_HEADER = 0xa5;
    const uint8_t FRAME_REFREE_TAILER = 0xff;



};


#endif