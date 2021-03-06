/*
  Manipulator Training 2018 Autumn
  Sample Program
  written by Rikuto SATO(2018/10)
*/

#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <math.h>

#include "ros/ros.h"
#include "ros/console.h"
#include "trajectory_msgs/JointTrajectory.h"

#define pi 3.1415926
#define VIA_POINT_NUM 30 //points
#define MOVING_TIME 3.0  //seconds

class PA10Controller
{
  public:
    void StartMoving();
    PA10Controller();

  private:
    ros::NodeHandle node_;     //node handler
    ros::Publisher joint_pub_; //define publisher

    /*Example definition of kinematics compute functions*/
    std::vector<double> ForwardKinematics(std::vector<double> &joint);
    std::vector<double> InverseKinematics(std::vector<double> &position);

    void TrajectoryGeneration();

    trajectory_msgs::JointTrajectory arm_; //joint state

    int ticks_ = 0;
};

//Constructor (if you do not know about c++ class, please search on the internet.)
PA10Controller::PA10Controller()
{
    //define ROS node
    joint_pub_ = node_.advertise<trajectory_msgs::JointTrajectory>(/*topic name=*/"/pa10/pa10_joint_controller/command", /*queue_size=*/50);

    //initiallize joint states
    arm_.joint_names.resize(7);
    arm_.joint_names[0] = "joint_1";
    arm_.joint_names[1] = "joint_2";
    arm_.joint_names[2] = "joint_3";
    arm_.joint_names[3] = "joint_4";
    arm_.joint_names[4] = "joint_5";
    arm_.joint_names[5] = "joint_6";
    arm_.joint_names[6] = "joint_7";
    //TODO: gazebo側をいじって、残りのエンドエフェクタの関節が動くようにしたい。
    // arm_.joint_names[7] = "gripper_finger";
    // arm_.joint_names[8] = "gripper_finger_mimic_joint";

    arm_.points.resize(VIA_POINT_NUM);

    //それぞれのpoints[]に対して、positions[]をリサイズしないといけないみたい。頭の悪いプログラムに見えるけど、仕方ない。
    for (int i = 0; i < VIA_POINT_NUM; i++)
    {
        arm_.points[i].positions.resize(7);
    }

    //Initialize Position (All angles are 0.0[rad])
    for (int i = 0; i < 7; i++)
    {
        arm_.points[0].positions[i] = 0.0;
    }

    arm_.header.stamp = ros::Time::now();
    arm_.header.frame_id = "";
}

//example template
std::vector<double> PA10Controller::ForwardKinematics(std::vector<double> &joint)
{

    //write by yourself
}

std::vector<double> PA10Controller::InverseKinematics(std::vector<double> &position)
{
    //write by yourself
    // note: there are multiple answers regarding to inverse kinematics.
}

void PA10Controller::TrajectoryGeneration()
{
    while (ticks_ < VIA_POINT_NUM)
    {
        double time;
        for (int i = 0; i < 7; i++)
        {
            arm_.points[ticks_].positions[i] = pi / 12 * i * ticks_ / VIA_POINT_NUM;
        }

        //"+1" here means that the arm is moving after 1 period of starting the program.
        time = (double)MOVING_TIME / VIA_POINT_NUM * (ticks_ + 1);
        arm_.points[ticks_].time_from_start = ros::Duration(time);

        ticks_++;
    }
}

void PA10Controller::StartMoving()
{
    //ros::Rate loop_rate(10); //set loop rate 10[Hz]

    TrajectoryGeneration();
    // Wait a little time to avoid error.
    ros::Duration(0.5).sleep();
    ROS_INFO("Press Enter to move");
    getc(stdin); //wait for keyboard input

    ROS_INFO("Start Moving");
    //publish joint states
    joint_pub_.publish(arm_);
    ROS_INFO("Published");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_state_pub_moveit");

    PA10Controller pa10controller;

    pa10controller.StartMoving();

    return 0;
}
