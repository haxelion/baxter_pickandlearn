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
    std::vector<Piece> pieces;
    BaxterController *robot = new BaxterController(nh);
    Camera  *camera =  new Camera(Camera::RIGHT_HAND, nh, pieces);
    int state = 0;
    float position[3], orientation[4];
    spinner.start();
    while(true)
    {
        BaxterController::ITBInput input = robot->getInput();
        if(state == 0 && input == BaxterController::INPUT_WHEEL_CLICKED)
        {
            camera->request(Camera::REQUEST_SELECTED_SHAPE);
            state = 1;
        }
        else if(state == 1 && camera->isResultAvailable())
        {
            std::vector<std::vector<cv::Point> > *result = camera->getResult();
            if(result->size() == 0 || (*result)[0].size() == 0)
            {
                std::cout << "No result" << std::endl;
                state = 0;
            }
            else
            {
                state = 2;
                robot->getPosition(position);
                robot->getOrientation(orientation);
                char buffer[32];
                snprintf(buffer, 32, "Piece %d", pieces.size()+1);
                pieces.push_back(Piece((*result)[0], position[3], orientation, std::string(buffer)));
                std::cout << pieces.back().getName() << " saved:" << std::endl;
                robot->grip();
            }
        }
        else if(state == 2 && input == BaxterController::INPUT_WHEEL_CLICKED)
        {
            state = 0;
            robot->release();
            robot->move(position, orientation);
        }
    }
    return 0;
}
