#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

std::string msg = R"(
Reading from the keyboard !
---------------------------
Moving around:
      q   w   e 
      a   s   d 
------'w' is moving forward
------'s' is moving backward
------'a' is moving left
------'d' is moving right
------'q' is rotating left
------'e' is rotating right
------'i' is increasing max speed by 10%
------'o' is decreasing max speed by 10%

anything else : stop

CTRL-C to quit
)";

std::map<char, std::tuple<double, double, double, double>> moveBindings = {
    //link_1  link_2  link_3  link_4 逆时针是1
    {'w', {-1, 1, -1, 1}},
    {'s', {1, -1, 1, -1}},
    {'a', {1, 1, -1, -1}},
    {'d', {-1, -1, 1, 1}},
    {'q', {1, 1, 1, 1}},
    {'e', {-1, -1, -1, -1}},
};

std::map<char, std::tuple<double, double>> speedBindings = {
    {'i', {1.1, 1.1}},
    {'o', {0.9, 0.9}},
};

char getKey() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    char c = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return c;
}

std::string vels(double speed) {
    return "currently:\tspeed " + std::to_string(speed) + " ";
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vel_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub_1_front = nh.advertise<std_msgs::Float64>("wheel_1_joint_velocity_controller/command", 1);
    ros::Publisher pub_2_front = nh.advertise<std_msgs::Float64>("wheel_2_joint_velocity_controller/command", 1);
    ros::Publisher pub_3_rear = nh.advertise<std_msgs::Float64>("wheel_3_joint_velocity_controller/command", 1);
    ros::Publisher pub_4_rear = nh.advertise<std_msgs::Float64>("wheel_4_joint_velocity_controller/command", 1);

    double speed = 1.0;
    double link_1 = 0;
    double link_2 = 0;
    double link_3 = 0;
    double link_4 = 0;
    int status = 0;

    std::cout << msg << std::endl;
    std::cout << vels(speed) << std::endl;

    while (ros::ok()) {
        char key = getKey();
        if (moveBindings.find(key) != moveBindings.end()) {
            std::tie(link_1, link_2, link_3, link_4) = moveBindings[key];
        } else if (speedBindings.find(key) != speedBindings.end()) {
            speed = speed * std::get<0>(speedBindings[key]);
            std::cout << vels(speed) << std::endl;
            if (status == 14) {
                std::cout << msg << std::endl;
            }
            status = (status + 1) % 15;
        } else {
            link_1 = 0;
            link_2 = 0;
            link_3 = 0;
            link_4 = 0;
            if (key == '\x03') {
                break;
            }
        }

        std_msgs::Float64 vel_1;
        std_msgs::Float64 vel_2;
        std_msgs::Float64 vel_3;
        std_msgs::Float64 vel_4;

        vel_1.data  = link_1 * speed;
        vel_2.data = link_2 * speed;
        vel_3.data   = link_3 * speed;
        vel_4.data  = link_4 * speed;

        pub_1_front.publish(vel_1);
        pub_2_front.publish(vel_2);
        pub_3_rear.publish(vel_3);
        pub_4_rear.publish(vel_4);
    }

    std_msgs::Float64 vel_1;
    std_msgs::Float64 vel_2;
    std_msgs::Float64 vel_3;
    std_msgs::Float64 vel_4;
    vel_1.data  = 0;
    vel_2.data = 0;
    vel_3.data   = 0;
    vel_4.data  = 0;

    pub_1_front.publish(vel_1);
    pub_2_front.publish(vel_2);
    pub_3_rear.publish(vel_3);
    pub_4_rear.publish(vel_4);

    return 0;
}