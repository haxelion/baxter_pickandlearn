#include <ros/ros.h>
#include "robot.h"
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
        if(input == BaxterController::INPUT_WHEEL_CLICKED)
        {
            camera->request(Camera::REQUEST_SELECTED_SHAPE);
            state = 1;
        }
        if(state == 1 && camera->isResultAvailable())
        {
            state = 0;
            std::vector<std::vector<cv::Point> > *result = camera->getResult();
            if(result == NULL)
                std::cout << "No result" << std::endl;
            else
            {
                std::cout << "Getting " << result->size() << " results" << std::endl;
                for(int i = 0; i<result->size(); i++)
                {
                    if((*result)[i].size() == 0)
                        continue;
                    std::cout << "Selected shape: [";
                    std::cout << "(" << (*result)[i][0].x << "," << (*result)[i][0].y << ")";
                    for(int j = 1; j<(*result)[i].size(); j++)
                        std::cout << ",(" << (*result)[i][j].x << "," << (*result)[i][j].y << ")";
                    std::cout << "]" << std::endl;
                }
            }
        }
    }
    return 0;
}
