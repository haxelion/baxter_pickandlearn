#include "baxtercontroller.h"

BaxterController::BaxterController(ros::NodeHandle nh)
{
    input = INPUT_NOTHING;
    gripper_hid = 0;
    last_input_time = clock();
    this->nh = nh;
    std::cout << "Registering ITB callbacks: ";
    itb_sub = nh.subscribe("/robot/itb/right_itb/state", 2, &BaxterController::itbCallback, this);
    if(itb_sub == NULL)
    {
        std::cout << std::right << std::setw(80) << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout  < std::right << std::setw(80) << "\033[1;32m[OK]\033[0m" << std::endl;
    std::cout << "Registering gripper callback: ";
    gripper_sub = nh.subscribe("/robot/end_effector/right_gripper/state", 2, &BaxterController::gripperCallback, this);
    if(gripper_sub == NULL)
    {
        std::cout << std::right << std::setw(80) << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout << "Registering gripper publisher: ";
    gripper_pub = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command", 2);
    if(gripper_pub == NULL)
    {
        std::cout << std::right << std::setw(80) << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout  < std::right << std::setw(80) << "\033[1;32m[OK]\033[0m" << std::endl;
    std::cout << "Registering IR subscriber: ";
    ir_sub = nh.subscribe("/robot/range/right_hand_range/state", 2, &BaxterController::irCallback, this);
    if(ir_sub == NULL)
    {
        std::cout << std::right << std::setw(80) << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout  < std::right << std::setw(80) << "\033[1;32m[OK]\033[0m" << std::endl;
    std::cout << "Registering endpoint subscriber: ";
    endpoint_sub = nh.subscribe("/robot/limb/right/endpoint_state", 2, &BaxterController::endpointCallback, this);
    if(endpoint_sub == NULL)
    {
        std::cout << std::right << std::setw(80) << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout  < std::right << std::setw(80) << "\033[1;32m[OK]\033[0m" << std::endl;
    std::cout << "Registering inverse kinematic solver client: ";
    ik_client = nh.serviceClient<SolvePositionIK>("ExternalTools/right/PositionKinematicsNode/IKService");
    if(ik_client == NULL)
    {
        std::cout << std::right << std::setw(80) << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout  < std::right << std::setw(80) << "\033[1;32m[OK]\033[0m" << std::endl;
    std::cout << "Registering joint publisher: ";
    joint_pub = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 2);
    if(joint_pub == NULL)
    {
        std::cout << std::right << std::setw(80) << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout  < std::right << std::setw(80) << "\033[1;32m[OK]\033[0m" << std::endl;
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

void BaxterController::move(float position[], float orientation[])
{
    baxter_core_msgs::SolverPositionIK srv;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "base";
    pose.stamped.pose.position[0] = position[0];
    pose.stamped.pose.position[1] = position[1];
    pose.stamped.pose.position[2] = position[2];
    pose.stamped.pose.orientation[0] = orientation[0];
    pose.stamped.pose.orientation[1] = orientation[1];
    pose.stamped.pose.orientation[2] = orientation[2];
    pose.stamped.pose.orientation[3] = orientation[3];
    srv.request.pose_stamp.push_back(geometry_msgs::PoseStamped());
    srv.request.pose_stamp.push_back(pose_stamped);
    ik_client.call(srv);
}
