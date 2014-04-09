#include "robot.h"

BaxterController::BaxterController(ros::NodeHandle nh)
{
    this->nh = nh;
    itb_sub = nh.subscribe("/robot/itb/right_itb/state", 2, &BaxterController::itbCallback, this);
    action = ACTION_NOTHING;
    end_effector_command.command = baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE; 
    std::cout << "Calibrating right hand gripper: ";
    if(!ros::service::call("/robot/end_effector/right_gripper/command", end_effector_command) || end_effector_command.response.err != 0)
    {
        std::cout << "\033[1;31mFailed\033[0m" << std::endl;
        return;
    }
    std::cout << "\033[1;32mOK\033[0m" << std::endl;
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
