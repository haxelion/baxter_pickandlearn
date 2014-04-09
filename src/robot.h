#include <ros/ros.h>
#include <baxter_core_msgs/ITBState.h>
#include <baxter_core_msgs/EndEffectorCommand.h>

class BaxterController
{
public:
    enum BaxterAction {ACTION_NOTHING, ACTION_WHEEL_CLICKED};
    BaxterController(ros::NodeHandle nh);
    ~BaxterController();
    void itbCallback(const baxter_core_msgs::ITBStateConstPtr &msg);
    BaxterAction getAction();

private:
    ros::NodeHandle nh;
    ros::Subscriber itb_sub;
    baxter_core_msgs::EndEffectorCommand end_effector_command;
    BaxterAction action;
};
