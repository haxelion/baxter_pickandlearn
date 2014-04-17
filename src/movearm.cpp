#include <ros/ros.h>
#include <vector>
#include "baxtercontroller.h"
#include "camera.h"

int main(int argc, char **argv)
{
    // Initialise ROS
    ros::init(argc, argv, "baxter_pickandlearn");
    ros::NodeHandle nh;
    BaxterController *robot = new BaxterController(nh);
    float position[3], orientation[4];
    while(true)
    {
        ros::spinOnce();
        robot->getOrientation(orientation);
        std::cin >> position[0] >> position[1] >> position[2];
        robot->move(position, orientation);
    }
    return 0;
}
