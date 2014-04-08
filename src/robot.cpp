#include "robot.h"

BaxterController::BaxterController(ros::NodeHandle nh)
{
    this->nh = nh;
    itb_sub = nh.subscribe("/robot/itb/right_itb/state", 2, &BaxterController::itbCallback, this);
    action = ACTION_NOTHING;
}

BaxterController::~BaxterController()
{
}

void BaxterController::itbCallback(const baxter_core_msgs::ITBStateConstPtr &msg)
{
    if(msg->buttons[0])
        action = ACTION_WHEEL_CLICKED;
}

BaxterController::BaxterAction BaxterController::getAction()
{
    BaxterAction t = action;
    action = ACTION_NOTHING;
    return t;
}
