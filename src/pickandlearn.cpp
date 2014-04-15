#include <ros/ros.h>
#include <vector>
#include "baxtercontroller.h"
#include "camera.h"
#include "object.cpp"

int main(int argc, char **argv)
{
    // Initialise ROS
    ros::init(argc, argv, "baxter_pickandlearn");
    ros::NodeHandle nh;
    BaxterController *robot = new BaxterController(nh);
    Camera  *camera =  new Camera(Camera::RIGHT_HAND, nh);
    std::vector<Object> *objects = new std::vector<Object>;
    camera->setHighlightObjects(objects);
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
            state = 2;
            std::vector<std::vector<cv::Point> > *result = camera->getResult();
            if(result->size() == 0 || (*result)[0].size() == 0)
                std::cout << "No result" << std::endl;
            else
            {
                float position[3];
                robot->getPosition(position);
                objects->push_back(new Object((*result)[0], position[3]));
                std::cout << "Object " << (objects.size()-1) << "saved" << std::endl;
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
