//
// Created by chen on 2021/8/10.
//
#include "detector_desktop/Arm.h"

#include <iostream>
#include <thread>

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <pcl/point_cloud.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <tf_conversions/tf_eigen.h>


using namespace std;

Arm::Arm(ros::NodeHandle* np_){
    arm_client.reset(new ArmClient(*np_, Config::kKinovaPoseActionAddress, true));
    finger_client.reset(new FingerClient(*np_, Config::kKinovaFingerActionAddress, true));
    transform_listener.reset(new tf::TransformListener);
    transform_broadcaster.reset(new tf::TransformBroadcaster);
    cv::cv2eigen(Config::kCamToArm, Tca.matrix());
    Eigen::AngleAxisd yawAngle(Config::kGraspPoseEuler.at<double>(0, 0), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(Config::kGraspPoseEuler.at<double>(1, 0), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rollAngle(Config::kGraspPoseEuler.at<double>(2, 0), Eigen::Vector3d::UnitX());
    grasp_q=yawAngle*pitchAngle*rollAngle; //先绕z旋转yaw,再绕y旋转pitch，最后绕x旋转roll
}


void Arm::GetGraspPose(Instance &inst, geometry_msgs::Pose &pose)
{
    Eigen::Quaterniond q;

    //抓取的位置应该是,根据机械臂的安装位置，x z轴均为中心点，物体的y轴为负的，因此需要+y_half
    auto minPt=inst.minPt;
    auto maxPt=inst.maxPt;
    float y_half_len=std::abs(std::abs(minPt.y)-std::abs(maxPt.y))/2.0f;//半径
    float z_half_len=std::abs(std::abs(minPt.z)-std::abs(maxPt.z))/2.0f;//半径
    Eigen::Vector3d center((minPt.x+maxPt.x)/2,(minPt.y+maxPt.y)/2,(minPt.z+maxPt.z)/2); //点云中心点
    pose.position.x=center.x();
    pose.position.y=center.y();//y_half_len+0.02f;//0.02f为余量
    pose.position.z=center.z()+z_half_len;

    //抓取方向设置:使用预先定义的方向
    pose.orientation.x=grasp_q.x();
    pose.orientation.y=grasp_q.y();
    pose.orientation.z=grasp_q.z();
    pose.orientation.w=grasp_q.w();
}

//发布抓取位置的tf，用于可视化
void Arm::BroadcastGraspPose(const geometry_msgs::Pose &pose)
{
    tf::Transform Ttf;
    tf::Quaternion q;
    q.setX(pose.orientation.x);
    q.setY(pose.orientation.y);
    q.setZ(pose.orientation.z);
    q.setW(pose.orientation.w);
    Ttf.setRotation(q);

    tf::Vector3 v;
    v.setX(pose.position.x);
    v.setY(pose.position.y);
    v.setZ(pose.position.z);
    Ttf.setOrigin(v);

    transform_broadcaster->sendTransform(
            tf::StampedTransform(Ttf, ros::Time::now(),
                                 "j2n6s300_link_base", "grasp_target_frame"));
}


bool Arm::SetFinger(float value){
    kinova_msgs::SetFingersPositionGoal goalFinger;
    goalFinger.fingers.finger1=value;//6120是%90的闭合，0是张到最大的状态
    goalFinger.fingers.finger2=value;
    goalFinger.fingers.finger3=value;
    finger_client->sendGoal(goalFinger);

    if(!finger_client->waitForResult(ros::Duration(5.0))){
        Debugt("fingerClient 超时");
        return false;
    }
    if(finger_client->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
        Debugt("fingerClient 失败,State:{}",finger_client->getState().toString());
        return false;
    }
    return true;
}


bool Arm::SetArmPose(geometry_msgs::Pose &pose)
{
    //设置目标
    kinova_msgs::ArmPoseGoal goalArm;
    goalArm.pose.header.frame_id=Config::kArmMsgFrame;
    goalArm.pose.pose=pose;
    arm_client->sendGoal(goalArm);

    if(!arm_client->waitForResult(ros::Duration(10.0))){
        Debugt("armClient 超时");
        return false;
    }
    if(arm_client->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
        Debugt("armClient 失败,State:{}",arm_client->getState().toString());
        return false;
    }
    return true;
}



bool Arm::Run(Instance inst)
{
    if(inst.pc->empty()){
        Debugt("instance未构建");
        return false;
    }

    //抓取点的设置
    geometry_msgs::Pose target_pose,pose;
    GetGraspPose(inst, target_pose);
    Debugt("抓取目标的三维位置：({},{},{})",target_pose.position.x,target_pose.position.y,target_pose.position.z);
    BroadcastGraspPose(target_pose);
    Debugt("Wait Action Server...");
    arm_client->waitForServer();
    finger_client->waitForServer();
    if(!SetFinger(0)) //打开手指
        return false;
    std::this_thread::sleep_for(500ms);//休眠
    pose=target_pose;
    pose.position.z +=0.1;
    if(!SetArmPose(pose)) //先到达目标上方
        return false;
    pose.position.z -=0.1;
    if(!SetArmPose(pose)) //到达目标所在的位置
        return false;
    std::this_thread::sleep_for(500ms);
    if(!SetFinger(Config::kArmFingerValue)) //关闭手指，抓取
        return false;
    std::this_thread::sleep_for(500ms);
    Debugt("已抓取物体 往回运动");
    pose.position.z+=0.1; //往上0.2m
    if(!SetArmPose(pose))
        return false;
    pose.position.x+=0.1; //往左0.1m
    if(!SetArmPose(pose))
        return false;
    Debugt("抓取线程完成");
    return true;
}


//使用future的方式判断程序机械臂是否正在运行
//这种方法虽然不是很直观
bool Arm::TryRunArm(Instance &inst) {
    if(!is_arm_running.valid()){ //第一次运行机械臂或结果已从future中取出
        is_arm_running=std::async(std::launch::async, &Arm::Run, this, inst);//立即启动
        return true;
    }
    else{
        if(is_arm_running.wait_for(std::chrono::milliseconds(1)) == std::future_status::ready){
            Debugt("前一次机械臂执行结果:{}",(is_arm_running.get() ? "成功" : "失败"));
            is_arm_running=std::async(std::launch::async, &Arm::Run, this, inst);
            return true;
        }
        else{
            return false;
        }
    }
}

