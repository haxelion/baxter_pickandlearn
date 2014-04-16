#include <time.h>
#include <ros/ros.h>
#include <baxter_core_msgs/ITBState.h>
#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/JointCommand.h>
#include <sensor_msgs/Range.h>

const clock_t INPUT_BLOCKING_TIME = CLOCKS_PER_SEC/2;

class BaxterController
{
public:
    enum ITBInput {INPUT_NOTHING, INPUT_WHEEL_CLICKED};
    BaxterController(ros::NodeHandle nh);
    ~BaxterController();
    void itbCallback(const baxter_core_msgs::ITBStateConstPtr &msg);
    void gripperCallback(const baxter_core_msgs::EndEffectorStateConstPtr &msg);
    void irCallback(const sensor_msgs::RangeConstPtr &msg);
    void endpointCallback(const baxter_core_msgs::EndpointStateConstPtr &msg);
    ITBInput getInput();
    float getRange();
    void getPosition(float position[]);
    void getOrientation(float orientation[]);
    void grip();
    void release();
    void move(float position[], float orientation[]);

private:
    ros::NodeHandle nh;
    ros::Subscriber itb_sub, gripper_sub, ir_sub, endpoint_sub;
    ros::Publisher gripper_pub, joint_pub;
    ros::ServiceClient ik_client;
    ITBInput input;
    clock_t last_input_time;
    unsigned int gripper_hid;
    float range;
    double position[3];
    double orientation[4];
};
