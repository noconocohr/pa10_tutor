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
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"

#define pi 3.1415926
#define VIA_POINT_NUM 30 //points
#define MOVING_TIME 3.0  //seconds

class PA10Controller
{
  public:
    void StartMoving();
    PA10Controller(); //constructor

  private:
    ros::NodeHandle node;     //node handler
    ros::Publisher joint_pub; //define publisher

    /*Example definition of kinematics compute functions*/
    std::vector<double> ForwardKinematics(sensor_msgs::JointState *joint); //forward kinematics
    //FIXME: メッセージ型を変えたので、関数の定義を変えたほうがいいかもね
    sensor_msgs::JointState InverseKinematics(std::vector<double> &position); //inverse kinematics

    void TrajectoryGeneration();
    void PathGenerate();

    trajectory_msgs::JointTrajectory arm; //joint state

    //TODO: ticksはクラス変数のままで良いのか？
    int ticks;
};

//Constructor (if you do not know about c++ class, please search on the internet.)
PA10Controller::PA10Controller()
{
    //define ROS node
    joint_pub = node.advertise<trajectory_msgs::JointTrajectory>("/pa10/pa10_joint_controller/command", 50);

    //initiallize joint states
    arm.joint_names.resize(7);
    arm.joint_names[0] = "joint_1";
    arm.joint_names[1] = "joint_2";
    arm.joint_names[2] = "joint_3";
    arm.joint_names[3] = "joint_4";
    arm.joint_names[4] = "joint_5";
    arm.joint_names[5] = "joint_6";
    arm.joint_names[6] = "joint_7";
    //TODO: gazebo側をいじって、残りのエンドエフェクタの関節が動くようにしたい。
    // arm.joint_names[7] = "gripper_finger";
    // arm.joint_names[8] = "gripper_finger_mimic_joint";

    arm.points.resize(VIA_POINT_NUM);

    arm.points[0].time_from_start = ros::Duration((double)1 / MOVING_TIME);

    //それぞれのpoints[]に対して、positions[]をリサイズしないといけないみたい。頭の悪いプログラムに見えるけど、仕方ない。
    for (int i = 0; i < VIA_POINT_NUM; i++)
    {
        arm.points[i].positions.resize(7);
    }

    //Initialize Position (All angles are 0.0[rad])
    for (int i = 0; i < 7; i++)
    {
        arm.points[0].positions[i] = 0.0;
    }

    arm.header.stamp = ros::Time::now();
    arm.header.frame_id = "";
}

/*example template
std::vector<double> PA10Controller::ForwardKinematics(sensor_msgs::JointState *joint)
{

//write by yourself

}


sensor_msgs::joint_state PA10Controller::InverseKinematics(std::vector<double>& position)
{
  //write by yourself
  // note: there are multiple answers regarding to inverse kinematics.
}


void PA10Controller::PathGenerate()
{
//write by yourself, by using 5-order interpolation
}
*/

void PA10Controller::TrajectoryGeneration()
{
    for (ticks = 0; ticks < VIA_POINT_NUM; ticks++)
    {
        double time;
        for (int i = 0; i < 7; i++)
        {
            arm.points[ticks].positions[i] = pi / 12 * i * ticks / VIA_POINT_NUM;
        }

        //"+1" here means that the arm is moving after 1 period of starting the program.
        time = (double)MOVING_TIME / VIA_POINT_NUM * (ticks + 1);
        arm.points[ticks].time_from_start = ros::Duration(time);
    }
}

void PA10Controller::StartMoving()
{
    //ros::Rate loop_rate(10); //set loop rate 10[Hz]
    ROS_INFO("Start Moving");

    TrajectoryGeneration();
    // Wait a little time to avoid error.
    ros::Duration(0.5).sleep();
    ROS_INFO("Press Enter to move");
    getc(stdin); //wait for keyboard input

    //publish joint states
    joint_pub.publish(arm);
    ROS_INFO("Published");
}

//main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_state_pub_moveit");

    PA10Controller pa10controller;

    pa10controller.StartMoving();

    return 0;
}
