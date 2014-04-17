#include <ros/ros.h>
#include <vector>
#include "baxtercontroller.h"
#include "camera.h"

int main(int argc, char **argv)
{
    // Initialise ROS
    ros::init(argc, argv, "baxter_pickandlearn");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    std::string command;
    bool running = true;
    BaxterController *robot = new BaxterController(nh);
    float position[3], orientation[4];
    spinner.start();
    while(running)
    {
        std::cin >> command;
        if(command == "pos")
        {
            robot->getPosition(position);
            std::cout << position[0] << " " << position[1] << " " <<position[2] << std::endl;
        }
        else if(command == "mov")
        {
            robot->getOrientation(orientation);
            std::cin >> position[0] >> position[1] >> position[2];
            robot->moveTo(position, orientation);
        }
        else if(command == "grp")
            robot->grip();
        else if(command == "rls")
            robot->release();
        else if(command == "bye")
            running = false;
        else
            std::cout << "Invalid command. List of command: pos, mov, grp, rls, bye." << std::endl;
    }
    spinner.stop();
    return 0;
}
