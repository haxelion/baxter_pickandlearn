#include "baxtercontroller.h"

BaxterController::BaxterController(ros::NodeHandle nh)
{
    input = INPUT_NOTHING;
    gripper_hid = 0;
    last_input_time = clock();
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
    std::cout << "Registrating IR subscriber: ";
    ir_sub = nh.subscribe("/robot/range/right_hand_range", 2, &BaxterController::irCallback, this);
    if(ir_sub == NULL)
    {
        std::cout << "\033[1;31mFailed\033[0m" << std::endl;
        return;
    }
    std::cout << "\033[1;32mOK\033[0m" << std::endl;
    std::cout << "Registrating endpoint subscriber: ";
    endpoint_sub = nh.subscribe("/robot/limb/right/endpoint_state", 2, &BaxterController::endpointCallback, this);
    if(endpoint_sub == NULL)
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
    if(clock() - last_input_time > INPUT_BLOCKING_TIME)
    {
        if(msg->buttons[0])
        {
            input = INPUT_WHEEL_CLICKED;
            last_input_time = clock();
        }
    }
}

void BaxterController::gripperCallback(const baxter_core_msgs::EndEffectorStateConstPtr &msg)
{
    if(gripper_hid == 0)
    {
        gripper_hid = msg->id;
        baxter_core_msgs::EndEffectorCommand cmd;
        cmd.id = gripper_hid;
        cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE;
        gripper_pub.publish(cmd);
    }
}

void BaxterController::irCallback(const sensor_msgs::RangeConstPtr &msg)
{
    this->range = msg->range;
}

void BaxterController::endpointCallback(const baxter_core_msgs::EndpointStateConstPtr &msg)
{
    position[0] = msg->pose.position.x;
    position[1] = msg->pose.position.y;
    position[2] = msg->pose.position.z;

    orientation[0] = msg->pose.orientation.x;
    orientation[1] = msg->pose.orientation.y;
    orientation[2] = msg->pose.orientation.z;
    orientation[3] = msg->pose.orientation.w;
}

float BaxterController::getRange()
{
    return range;
}

void BaxterController::getPosition(float position[])
{
    position[0] = this->position[0];
    position[1] = this->position[1];
    position[2] = this->position[2];
}

void BaxterController::getOrientation(float orientation[])
{
    orientation[0] = this->orientation[0];
    orientation[1] = this->orientation[1];
    orientation[2] = this->orientation[2];
    orientation[3] = this->orientation[3];
}

BaxterController::ITBInput BaxterController::getInput()
{
    ITBInput t = input;
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
