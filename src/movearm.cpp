#include <ros/ros.h>
#include <vector>
#include "baxtercontroller.h"
#include "camera.h"

int main(int argc, char **argv)
{
    // Initialise ROS
    ros::init(argc, argv, "baxter_pickandlearn");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    BaxterController *robot = new BaxterController(nh);
    float position[3], orientation[4];
    robot->getOrientation(orientation);
    spinner.start();
    while(!(std::cin.eof()))
    {
        std::cin >> position[0] >> position[1] >> position[2];
        robot->move(position, orientation);
    }
    spinner.stop();
    return 0;
}
