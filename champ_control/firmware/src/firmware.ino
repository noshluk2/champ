#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <champ_msgs/Point.h>
#include <champ_msgs/PointArray.h>
#include <champ_msgs/Joints.h>
#include <champ_msgs/Pose.h>
#include <champ_description.h>
#include <champ_config.h>
#include <quadruped_ik.h>
#include <quadruped_balancer.h>
#include <quadruped_gait.h>

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;

unsigned long g_prev_command_time = 0;

void commandCallback(const geometry_msgs::Twist& cmd_msg);

ros::NodeHandle nh;
champ_msgs::PointArray point_msg;
ros::Publisher point_pub("/champ/foot/raw", &point_msg);

champ_msgs::Joints joints_msg;
ros::Publisher jointstates_pub("/champ/joint_states/raw", &joints_msg);

champ_msgs::Pose pose_msg;
ros::Publisher pose_pub("/champ/pose/raw", &pose_msg);

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);

QuadrupedBase base(lf_leg, rf_leg, lh_leg, rh_leg);
QuadrupedBalancer balancer(base);
QuadrupedIK ik(base);
QuadrupedGait gait(base, MAX_VELOCITY, SWING_HEIGHT, STEP_LENGTH, STANCE_DEPTH);

void setup()
{
    joints_msg.position_length = 12;

    nh.initNode();
    nh.getHardware()->setBaud(115200);
    // nh.advertise(point_pub);
    nh.advertise(jointstates_pub);
    nh.advertise(pose_pub);
    nh.subscribe(cmd_sub);

    while (!nh.connected())
    {
        nh.spinOnce();
    }

    nh.loginfo("CHAMP CONNECTED");
    delay(1);
}

void loop() { 
    static unsigned long prev_control_time = 0;

    if ((micros() - prev_control_time) >= 10000)
    {
        Transformation foot_positions[4];
        float joint_positions[12]; 

        base.lf->joints(0, 0, 0);
        base.rf->joints(0, 0, 0);
        base.lh->joints(0, 0, 0);
        base.rh->joints(0, 0, 0);
        base.attitude(0.0, 0.0, 0.0);

        balancer.balance(foot_positions, 0.0, 0.0, 0.0, 0.0, 0.0, -0.08);
        gait.generate(foot_positions, g_req_linear_vel_x,  g_req_linear_vel_y, g_req_angular_vel_z);
        ik.solve(foot_positions, joint_positions);

        // publishPoints(foot_positions);
        publishPose(0, 0, base.lf->nominal_stance().X() - 0.08, 0, 0, 0);
        publishJointStates(joint_positions);

        prev_control_time = micros();
    }

    if ((micros() - g_prev_command_time) >= (0.5 * SECONDS_TO_MICROS))
    {
        stopBase();
    }

    nh.spinOnce();
}

void stopBase()
{
    g_req_linear_vel_x = 0;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = 0;
}

void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
    g_req_linear_vel_x = cmd_msg.linear.x;
    g_req_linear_vel_y = cmd_msg.linear.y;
    g_req_angular_vel_z = cmd_msg.angular.z;

    g_prev_command_time = micros();
}

void publishJointStates(float joint_positions[12])
{
    joints_msg.position = joint_positions;
    jointstates_pub.publish(&joints_msg);  
}

void publishPoints(Transformation foot_positions[4])
{
    point_msg.lf.x = foot_positions[0].X();
    point_msg.lf.y = foot_positions[0].Y();
    point_msg.lf.z = foot_positions[0].Z();

    point_msg.rf.x = foot_positions[1].X();
    point_msg.rf.y = foot_positions[1].Y();
    point_msg.rf.z = foot_positions[1].Z();

    point_msg.lh.x = foot_positions[2].X();
    point_msg.lh.y = foot_positions[2].Y();
    point_msg.lh.z = foot_positions[2].Z();

    point_msg.rh.x = foot_positions[3].X();
    point_msg.rh.y = foot_positions[3].Y();
    point_msg.rh.z = foot_positions[3].Z();

    point_pub.publish(&point_msg);
}

void publishPose(float x, float y, float z, float roll, float pitch, float yaw)
{
    pose_msg.x = x;
    pose_msg.y = y;
    pose_msg.z = z;
    pose_msg.roll = roll;
    pose_msg.pitch = pitch;
    pose_msg.yaw = yaw;

    pose_pub.publish(&pose_msg);
}