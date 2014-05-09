#include <ros/ros.h>
#include <vector>
#include <math.h>
#include "baxtercontroller.h"
#include "camera.h"

float distance(float p1[], float p2[], float o1[], float o2[])
{
    float d = 0;
    for(int i = 0; i < 3; i++)
        d += (p1[i]-p2[i])*(p1[i]-p2[i]);
    for(int i = 0; i< 4; i++)
        d += (o1[i]-o2[i])*(o1[i]-o2[i]);
    return sqrt(d);
}

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
    float obs_position[3] = {0.7,0,0.15};
    float obs_orientation[4] = {1,0,0,0};
    float position[3], obj_position[3], orientation[4], obj_orientation[4];
    spinner.start();
    // Learning
    bool learning = true;
    while(learning)
    {
        BaxterController::ITBInput input = robot->getInput();
        if(input == BaxterController::INPUT_HOME_CLICKED)
            learning = false;
        if(state == 0)
        {
            if(input ==  BaxterController::INPUT_WHEEL_CLICKED)
            {
                camera->request(Camera::REQUEST_SELECTED_SHAPE);
                state = 1;
            }
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
                pieces.push_back(Piece((*result)[0], position[2], orientation, std::string(buffer)));
                std::cout << pieces.back().getName() << " saved:" << std::endl;
                robot->grip();
            }
            delete result;
        }
        else if(state == 2 && input == BaxterController::INPUT_WHEEL_CLICKED)
        {
            robot->getPosition(position);
            robot->getOrientation(orientation);
            pieces.back().setDropPosition(position, orientation);
            state = 0;
            robot->release();
        }
        ros::Duration(0.1).sleep();
    }
    bool sorting = true;
    state = 0;
    while(sorting)
    {
        int matchs, matchp;
        float x, y, dz;
        BaxterController::ITBInput input = robot->getInput();
        if(input == BaxterController::INPUT_HOME_CLICKED)
            sorting = false;
        if(state == 0)
        {
            robot->getPosition(position);
            robot->getOrientation(orientation);
            if(distance(position, obs_position, orientation, obs_orientation) < 0.01)
                state = 1;
            else
                robot->moveTo(obs_position, obs_orientation);
        }
        if(state == 1)
        {
            camera->request(Camera::REQUEST_SHAPES);
            state = 2;
        }
        else if(state == 2 && camera->isResultAvailable())
        {
            std::vector<std::vector<cv::Point> > *result = camera->getResult();
            if(result->size() == 0 || (*result)[0].size() == 0)
            {
                std::cout << "No shape found" << std::endl;
                state = 1;
            }
            else
            {
                closestMatch(pieces, (*result), 0.05, matchp, matchs);
                if(matchs == -1 || matchp == -1)
                {
                    std::cout << "No piece found." << std::endl;
                    state = 1;
                }
                else
                {
                    cv::Moments m = cv::moments((*result)[matchs]);
                    robot->getPosition(position);
                    x = m.m10/m.m00;
                    y = m.m01/m.m00;
                    std::cout << "Found at X: " << x << " Y: " << y << std::endl;
                    dz = position[2]-pieces[matchp].getPickingHeight();
                    camera->setAim((int) x, (int) y);
                    x -= 640;
                    y -= 400;
                    camera->cameraTransform(x, y, dz);
                    obj_position[0] = position[0]+y+0.01;
                    obj_position[1] = position[1]+x+0.03;
                    obj_position[2] = position[2]-dz+0.05;
                    pieces[matchp].getPickingOrientation(obj_orientation);
                    state = 3;
                    if(robot->moveTo(obj_position, obj_orientation))
                        state = 0;
                }
            }
            delete result;
        }
        else if(state == 3)
        {
            robot->getPosition(position);
            robot->getOrientation(orientation);
            if(distance(position, obj_position, orientation, obj_orientation) < 0.01)
            {
                state = 4;
                obj_position[2] -= 0.05;
                if(robot->moveTo(obj_position, obj_orientation))
                    state = 0;
            }
        }
        else if(state == 4)
        {
            robot->getPosition(position);
            robot->getOrientation(orientation);
            if(distance(position, obj_position, orientation, obj_orientation) < 0.01)
            {
                robot->grip();
                ros::Duration(0.5).sleep();
                obj_position[2] += dz;
                state = 5;
                if(robot->moveTo(obj_position, obj_orientation))
                    state = 0;
            }
        }
        else if (state == 5)
        {
            robot->getPosition(position);
            robot->getOrientation(orientation);
            if(distance(position, obj_position, orientation, obj_orientation) < 0.01)
            {
                robot->release();
                state = 0;
            }
        }
        ros::Duration(0.1).sleep();
    }
    spinner.stop();
    delete robot;
    delete camera;
    return 0;
}
