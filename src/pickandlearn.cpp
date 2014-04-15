#include <ros/ros.h>
#include "baxtercontroller.h"
#include "camera.h"

int main(int argc, char **argv)
{
    // Initialise ROS
    ros::init(argc, argv, "baxter_pickandlearn");
    ros::NodeHandle nh;
    BaxterController *robot = new BaxterController(nh);
    Camera  *camera =  new Camera(Camera::RIGHT_HAND, nh);
    int state = 0;
    while(true)
    {
        ros::spinOnce();
        BaxterController::ITBInput input = robot->getInput();
        if(state == 0 && input == BaxterController::INPUT_WHEEL_CLICKED)
        {
            camera->request(Camera::REQUEST_SELECTED_SHAPE);
            state = 1;
        }
        else if(state == 1 && camera->isResultAvailable())
        {
            float position[3], orientation[4];
            state = 2;
            robot->getPosition(position);
            robot->getOrientation(position);
            std::cout << "Position:";
            for(int i = 0; i<3; i++)
                std::cout << " " << position[i];
            std::cout << std::endl;
            std::cout << "Orientation:";
            for(int i = 0; i<3; i++)
                std::cout << " " << orientation[i];
            std::cout << std::endl;
            std::cout << "Range: " << robot->getRange() << std::endl;
            std::vector<std::vector<cv::Point> > *result = camera->getResult();
            if(result->size() == 0 || (*result)[0].size() == 0)
                std::cout << "No result" << std::endl;
            else
            {
                std::cout << "Selected shape: [";
                std::cout << "(" << (*result)[0][0].x << "," << (*result)[0][0].y << ")";
                for(int j = 1; j<(*result)[0].size(); j++)
                    std::cout << ",(" << (*result)[0][j].x << "," << (*result)[0][j].y << ")";
                std::cout << "]" << std::endl;
                robot->grip();
            }
        }
        else if(state == 2 && input == BaxterController::INPUT_WHEEL_CLICKED)
        {
            state = 0;
            robot->release();
        }
    }
    return 0;
}
