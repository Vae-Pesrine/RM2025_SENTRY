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
    //left_front  right_front  left_rear  right_rear 顺时针是1
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

    ros::Publisher pub_left_front = nh.advertise<std_msgs::Float64>("/open_base/left_front_joint_velocity_controller/command", 1);
    ros::Publisher pub_right_front = nh.advertise<std_msgs::Float64>("/open_base/right_front_joint_velocity_controller/command", 1);
    ros::Publisher pub_left_rear = nh.advertise<std_msgs::Float64>("/open_base/left_rear_joint_velocity_controller/command", 1);
    ros::Publisher pub_right_rear = nh.advertise<std_msgs::Float64>("/open_base/right_rear_joint_velocity_controller/command", 1);

    double speed = 1.0;
    double left_front = 0;
    double right_front = 0;
    double left_rear = 0;
    double right_rear = 0;
    int status = 0;

    std::cout << msg << std::endl;
    std::cout << vels(speed) << std::endl;

    while (ros::ok()) {
        char key = getKey();
        if (moveBindings.find(key) != moveBindings.end()) {
            std::tie(left_front, right_front, left_rear, right_rear) = moveBindings[key];
        } else if (speedBindings.find(key) != speedBindings.end()) {
            speed = speed * std::get<0>(speedBindings[key]);
            std::cout << vels(speed) << std::endl;
            if (status == 14) {
                std::cout << msg << std::endl;
            }
            status = (status + 1) % 15;
        } else {
            left_front = 0;
            right_front = 0;
            left_rear = 0;
            right_rear = 0;
            if (key == '\x03') {
                break;
            }
        }

        std_msgs::Float64 vel_left_front;
        std_msgs::Float64 vel_right_front;
        std_msgs::Float64 vel_left_rear;
        std_msgs::Float64 vel_right_rear;

        vel_left_front.data  = left_front * speed;
        vel_right_front.data = right_front * speed;
        vel_left_rear.data   = left_rear * speed;
        vel_right_rear.data  = right_rear * speed;

        pub_left_front.publish(vel_left_front);
        pub_right_front.publish(vel_right_front);
        pub_left_rear.publish(vel_left_rear);
        pub_right_rear.publish(vel_right_rear);
    }

    std_msgs::Float64 vel_left_front;
    std_msgs::Float64 vel_right_front;
    std_msgs::Float64 vel_left_rear;
    std_msgs::Float64 vel_right_rear;
    vel_left_front.data  = 0;
    vel_right_front.data = 0;
    vel_left_rear.data   = 0;
    vel_right_rear.data  = 0;

    pub_left_front.publish(vel_left_front);
    pub_right_front.publish(vel_right_front);
    pub_left_rear.publish(vel_left_rear);
    pub_right_rear.publish(vel_right_rear);

    return 0;
}