#include <time.h>
#include <ros/ros.h>
#include <baxter_core_msgs/ITBState.h>
#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/EndEffectorCommand.h>

const clock_t INPUT_BLOCKING_TIME = CLOCKS_PER_SEC/2;

class BaxterController
{
public:
    enum ITBInput {INPUT_NOTHING, INPUT_WHEEL_CLICKED};
    BaxterController(ros::NodeHandle nh);
    ~BaxterController();
    void itbCallback(const baxter_core_msgs::ITBStateConstPtr &msg);
    void gripperCallback(const baxter_core_msgs::EndEffectorState &msg);
    ITBInput getInput();
    void grip();
    void release();

private:
    ros::NodeHandle nh;
    ros::Subscriber itb_sub, gripper_sub;
    ros::Publisher gripper_pub;
    ITBInput input;
    clock_t last_input_time;
    unsigned int gripper_hid;
};
