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
    //front_left  front_right  back_left  back_right
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

    ros::Publisher pub_front_left = nh.advertise<std_msgs::Float64>("/open_base/front_left_joint_velocity_controller/command", 1);
    ros::Publisher pub_front_right = nh.advertise<std_msgs::Float64>("/open_base/front_right_joint_velocity_controller/command", 1);
    ros::Publisher pub_back_left = nh.advertise<std_msgs::Float64>("/open_base/back_left_joint_velocity_controller/command", 1);
    ros::Publisher pub_back_right = nh.advertise<std_msgs::Float64>("/open_base/back_right_joint_velocity_controller/command", 1);

    double speed = 1.0;
    double front_left = 0;
    double front_right = 0;
    double back_left = 0;
    double back_right = 0;
    int status = 0;

    std::cout << msg << std::endl;
    std::cout << vels(speed) << std::endl;

    while (ros::ok()) {
        char key = getKey();
        if (moveBindings.find(key) != moveBindings.end()) {
            std::tie(front_left, front_right, back_left, back_right) = moveBindings[key];
        } else if (speedBindings.find(key) != speedBindings.end()) {
            speed = speed * std::get<0>(speedBindings[key]);
            std::cout << vels(speed) << std::endl;
            if (status == 14) {
                std::cout << msg << std::endl;
            }
            status = (status + 1) % 15;
        } else {
            front_left = 0;
            front_right = 0;
            back_left = 0;
            back_right = 0;
            if (key == '\x03') {
                break;
            }
        }

        std_msgs::Float64 vel_front_left;
        std_msgs::Float64 vel_front_right;
        std_msgs::Float64 vel_back_left;
        std_msgs::Float64 vel_back_right;

        vel_front_left.data  = front_left * speed;
        vel_front_right.data = front_right * speed;
        vel_back_left.data   = back_left * speed;
        vel_back_right.data  = back_right * speed;

        pub_front_left.publish(vel_front_left);
        pub_front_right.publish(vel_front_right);
        pub_back_left.publish(vel_back_left);
        pub_back_right.publish(vel_back_right);
    }

    std_msgs::Float64 vel_front_left;
    std_msgs::Float64 vel_front_right;
    std_msgs::Float64 vel_back_left;
    std_msgs::Float64 vel_back_right;
    vel_front_left.data  = 0;
    vel_front_right.data = 0;
    vel_back_left.data   = 0;
    vel_back_right.data  = 0;

    pub_front_left.publish(vel_front_left);
    pub_front_right.publish(vel_front_right);
    pub_back_left.publish(vel_back_left);
    pub_back_right.publish(vel_back_right);

    return 0;
}