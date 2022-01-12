/*******************************************************
 *
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of detector_desktop.
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef DETECTOR_ARM_H
#define DETECTOR_ARM_H

#include <future>

#include <Eigen/Geometry>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <kinova_msgs/ArmPoseGoal.h>
#include <kinova_msgs/ArmPoseAction.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <kinova_msgs/SetFingersPositionGoal.h>

#include "detector_desktop/Types.h"
#include "detector_desktop/Config.h"

class Arm{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Arm> Ptr;
    typedef actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> ArmClient;
    typedef actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> FingerClient;
    explicit Arm(ros::NodeHandle *np_);
    bool TryRunArm(Instance &inst);
private:
    bool Run(Instance inst);
    void GetGraspPose(Instance &inst, geometry_msgs::Pose &pose);
    void BroadcastGraspPose(const geometry_msgs::Pose &pose);
    bool SetFinger(float value);
    bool SetArmPose(geometry_msgs::Pose &pose);

    std::shared_ptr<ArmClient> arm_client;
    std::shared_ptr<FingerClient> finger_client;
    std::future<bool> is_arm_running;//用于返回异步调用的执行结果
    Eigen::Affine3d Tca;
    Eigen::Quaterniond grasp_q;
    std::shared_ptr<tf::TransformListener> transform_listener;
    std::shared_ptr<tf::TransformBroadcaster> transform_broadcaster;
};


#endif //DETECTOR_ARM_H
