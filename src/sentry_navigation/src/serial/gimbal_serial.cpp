#include <vector>
#include "gimbal_serial.hpp"

sentrySerial::sentrySerial(): pr_nh_("~")
{
    pr_nh_.param<std::string>("serial_port", serial_port_, "/dev/ttyUSB0");
    pr_nh_.param<uint32_t>("serial_baudrate", serial_baudrate_, 115200);

    gimbal_send_ = new gimbal_send_msg();
    gimbal_send_->header = FRAME_REFREE_HEADER;
    gimbal_send_->tailer = FRAME_REFREE_TAILER;

    openSerial(gimbal_serial_, serial_port_, serial_baudrate_);

    sub_vel_ = nh_.subscribe("/cmd_vel", 10, &sentrySerial::cmdVelCallback, this);
    pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, this);
    pub_refree_ = nh_.advertise<sentry_navigation::refree>("/refree_msg", 1, this);
}

void sentrySerial::openSerial(serial::Serial &serial, std::string &serial_port, uint32_t &serial_baudrate)
{
    serial.setPort(serial_port.c_str());
    serial.setBaudrate(serial_baudrate);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(1000);  
    serial.setTimeout(time_out);

    try{
        serial.open();
    } catch(const std::exception& e){
        ROS_INFO_STREAM("Serial port cann't be opened!");
        return;
    }

    if(serial.isOpen()){
        ROS_INFO_STREAM("Serial port initialized!");
    } else{
        return;
    }
}

void sentrySerial::cmdVelCallback(geometry_msgs::TwistConstPtr msg)
{
    gimbal_send_->v_x = msg->linear.x;
    gimbal_send_->v_y = msg->linear.y;
    gimbal_send_->w_z = msg->angular.z;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gimbal_serial");
    sentrySerial sentrySerial;
    ros::Rate loop_rate(50);


    int count = 0;
    while(ros::ok())
    {
        //read port
        if(sentrySerial.gimbal_serial_.available() >= sizeof(sentry_navigation::refree))
        {
            std::vector<int> header_pos;
            std::vector<int> tailer_pos;
            int head_pos;
            int tail_pos;
            sentrySerial.gimbal_serial_.read(sentrySerial.DATA_RX_BUFF, sizeof(sentry_navigation::refree));
            // for(int i = 0;i<sizeof(usr_fun::refree);i++)
            // {
            //     printf("%02X",static_cast<unsigned char>(rx_buffer[i]));
            // }
            // printf("\n");
            for(int i = 0;i<sizeof(sentry_navigation::refree);i++){
                if(sentrySerial.DATA_RX_BUFF[i] == sentrySerial.FRAME_REFREE_HEADER){
                    head_pos = i;
                }
                if(sentrySerial.DATA_RX_BUFF[i] == sentrySerial.FRAME_REFREE_TAILER){
                    tail_pos = i;
                }
            }
            unsigned char temp_buffer[sizeof(sentry_navigation::refree)];
            for(int i=0;i<sizeof(sentry_navigation::refree);i++){
                temp_buffer[i] = sentrySerial.DATA_RX_BUFF[(head_pos + i)%sizeof(sentry_navigation::refree)];
            }
            memcpy(&sentrySerial.refree_msg_, temp_buffer,sizeof(sentry_navigation::refree));
            ROS_INFO("publish refree");
            sentrySerial.pub_refree_.publish(sentrySerial.refree_msg_);
        }

        if(count == 0)
        {   
            count++;
            if(count>=500)  count=0;

            memcpy(sentrySerial.DATA_TX_BUFF, sentrySerial.gimbal_send_, sizeof(gimbal_send_msg));
            uint8_t * a = (uint8_t *)sentrySerial.DATA_TX_BUFF;
            sentrySerial.gimbal_serial_.write((const uint8_t *)sentrySerial.DATA_TX_BUFF,sizeof(gimbal_send_msg));    

            ros::spinOnce();
            loop_rate.sleep();
        }
    return 0;
}
}