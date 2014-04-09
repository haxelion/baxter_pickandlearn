#include "robot.h"

BaxterController::BaxterController(ros::NodeHandle nh)
{
    this->nh = nh;
    std::cout << "Registrating ITB callbacks: ";
    itb_sub = nh.subscribe("/robot/itb/right_itb/state", 2, &BaxterController::itbCallback, this);
    if(itb_sub == NULL)
    {
        std::cout << "\033[1;31mFailed\033[0m" << std::endl;
        return;
    }
    std::cout << "\033[1;32mOK\033[0m" << std::endl;
    std::cout << "Registrating gripper callback: ";
    gripper_sub = nh.subscribe("/robot/end_effector/right_gripper/state", 2, &BaxterController::gripperCallback, this);
    if(gripper_sub == NULL)
    {
        std::cout << "\033[1;31mFailed\033[0m" << std::endl;
        return;
    }
    std::cout << "\033[1;32mOK\033[0m" << std::endl;
    std::cout << "Registrating gripper publisher: ";
    gripper_pub = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command", 2);
    if(gripper_pub == NULL)
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
        action = INPUT_WHEEL_CLICKED;
}

void BaxterController::gripperCallback(const baxter_core_msgs::EndEffectorState &msg)
{
    if(gripper_hid == 0)
    {
        gripper_hid = msg.id;
        baxter_core_msgs::EndEffectorCommand cmd;
        cmd.id = gripper_hid;
        cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE;
        gripper_pub.publish(cmd);
    }
}

BaxterController::ITBInput BaxterController::getInput()
{
    BaxterITB t = input;
    input = INPUT_NOTHING;
    return t;
}

void BaxterController::grip()
{
    baxter_core_msgs::EndEffectorCommand cmd;
    cmd.id = gripper_hid;
    cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_GRIP;
    gripper_pub.publish(cmd);
}

void BaxterController::release()
{
    baxter_core_msgs::EndEffectorCommand cmd;
    cmd.id = gripper_hid;
    cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_RELEASE;
    gripper_pub.publish(cmd);
}
