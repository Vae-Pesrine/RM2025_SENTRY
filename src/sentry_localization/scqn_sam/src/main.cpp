#include "scqn_sam/scqn_sam.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scqn_sam_node");
    ros::NodeHandle pr_nh("~");

    ScqnSam scqn_sam(pr_nh);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    scqn_sam.~ScqnSam();
    return 0;
}