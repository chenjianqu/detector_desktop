/*******************************************************
 *
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of detector_desktop.
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "detector_desktop/utils.h"
#include <geometry_msgs/Pose.h>
#include "detector_desktop/Config.h"


void BuildLineStripMarker(Instance &inst,visualization_msgs::Marker &msg){

    msg.header.frame_id=Config::kPointCloudMsgFrame;
    msg.header.stamp=ros::Time::now();
    msg.ns="box_strip";
    msg.action=visualization_msgs::Marker::ADD;
    msg.pose.orientation.w=1.0;
    //暂时使用类别代替这个ID
    msg.id=inst.box.classId;//当存在多个marker时用于标志出来
    //cout<<msg.id<<endl;
    msg.lifetime=ros::Duration(4);//持续时间3s，若为ros::Duration()表示一直持续
    msg.type=visualization_msgs::Marker::LINE_STRIP;//marker的类型
    msg.scale.x=0.01;//线宽
    msg.color.r=1.0;msg.color.g=1.0;msg.color.b=1.0;
    msg.color.a=1.0;//不透明度
    //设置立方体的八个顶点
    geometry_msgs::Point p[8];
    p[0].x=inst.minPt.x;p[0].y=inst.minPt.y;p[0].z=inst.minPt.z;
    p[1].x=inst.maxPt.x;p[1].y=inst.minPt.y;p[1].z=inst.minPt.z;
    p[2].x=inst.maxPt.x;p[2].y=inst.minPt.y;p[2].z=inst.maxPt.z;
    p[3].x=inst.minPt.x;p[3].y=inst.minPt.y;p[3].z=inst.maxPt.z;
    p[4].x=inst.minPt.x;p[4].y=inst.maxPt.y;p[4].z=inst.maxPt.z;
    p[5].x=inst.maxPt.x;p[5].y=inst.maxPt.y;p[5].z=inst.maxPt.z;
    p[6].x=inst.maxPt.x;p[6].y=inst.maxPt.y;p[6].z=inst.minPt.z;
    p[7].x=inst.minPt.x;p[7].y=inst.maxPt.y;p[7].z=inst.minPt.z;
    //这个类型仅将相邻点进行连线
    for(auto &pt : p)
        msg.points.push_back(pt);
    //为了保证矩形框的其它边存在：
    msg.points.push_back(p[0]);
    msg.points.push_back(p[3]);
    msg.points.push_back(p[2]);
    msg.points.push_back(p[5]);
    msg.points.push_back(p[6]);
    msg.points.push_back(p[1]);
    msg.points.push_back(p[0]);
    msg.points.push_back(p[7]);
    msg.points.push_back(p[4]);
}

void BuildTextMarker(Instance &inst,visualization_msgs::Marker &msg)
{
    msg.header.frame_id=Config::kPointCloudMsgFrame;
    msg.header.stamp=ros::Time::now();
    msg.ns="box_text";
    msg.action=visualization_msgs::Marker::ADD;
    //暂时使用类别代替这个ID
    msg.id=inst.box.classId+1000;//当存在多个marker时用于标志出来
    msg.lifetime=ros::Duration(4);//持续时间4s，若为ros::Duration()表示一直持续
    msg.type=visualization_msgs::Marker::TEXT_VIEW_FACING;//marker的类型
    msg.scale.z=0.1;//字体大小
    msg.color.r=1.0;msg.color.g=1.0;msg.color.b=1.0;
    msg.color.a=1.0;//不透明度
    geometry_msgs::Pose pose;
    pose.position.x=inst.maxPt.x;pose.position.y=inst.maxPt.y;pose.position.z=inst.maxPt.z;
    pose.orientation.w=1.0;
    msg.pose=pose;
    msg.text= inst.box.GetClassLabel().c_str();//c_str()是必要的
}



