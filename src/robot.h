#include <baxter_core_msgs/ITBState.h>

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
    BaxterAction action;
};
