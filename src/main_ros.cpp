/*******************************************************
 *
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of detector_desktop.
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include <string>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "detector_desktop/Config.h"
#include "detector_desktop/Detector.h"
#include "detector_desktop/Dataloader.h"
#include "detector_desktop/Segmentor.h"
#include "detector_desktop/Arm.h"
#include "detector_desktop/utils.h"

using namespace std;


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;

message_filters::Synchronizer<sync_pol>  *image_sub_ptr;
std::shared_ptr<ros::Subscriber> image_info_sub;

Arm::Ptr arm;
Dataloader::Ptr dataloader;
Frame::Ptr current;

void MouseCallback(int event,int x,int y,int flags,void *p)
{
    if(event==CV_EVENT_FLAG_LBUTTON){//鼠标左键
        for(auto &inst : current->insts){
            if(inst.box.IsBelong(x,y)){//点击在这个包围框内
                Debugt("尝试抓取·{}",Box2D::class_names[inst.box.classId]);
                if(!arm->TryRunArm(inst))
                    Debugt("尝试抓取失败，机械臂正在运行...");
                break;//一次只抓取一个就好了
            }
        }
    }
}

void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD) {
    cv_bridge::CvImageConstPtr rgb_ptr;
    try{
        rgb_ptr = cv_bridge::toCvShare(msgRGB);
    }catch (cv_bridge::Exception& e){
        Errort("cv_bridge exception: {}", e.what());
        return;
    }
    cv_bridge::CvImageConstPtr depth_ptr;
    try{
        depth_ptr = cv_bridge::toCvShare(msgD);
    }catch (cv_bridge::Exception& e){
        Errort("cv_bridge exception: {}", e.what());
        return;
    }
    cv::Mat bgr;
    cv::cvtColor(rgb_ptr->image,bgr,CV_RGB2BGR);
    dataloader->PushBack(bgr, depth_ptr->image.clone());
}

void InfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(info_msg);
    image_info_sub->shutdown();
    Config::kFX=cam_model.fx();
    Config::kFY=cam_model.fy();
    Config::kCX=cam_model.cx();
    Config::kCY=cam_model.cy();
    image_sub_ptr->registerCallback(boost::bind(GrabRGBD,_1,_2));
    Debugt("相机参数获取成功:\n cx:{},cy:{},fx:{},fy:{}", Config::kCX, Config::kCY, Config::kFX, Config::kFY);
}


int main(int argc, char** argv)
{
    if(argc!=2){
        std::cerr<<"please set config.yaml";
        return -1;
    }

    setlocale(LC_ALL, "");//防止中文乱码
    ros::init(argc, argv, "detect_node");
    ros::start();
    ros::NodeHandle nh;

    std::string config_file(argv[1]);
    Config cfg(config_file);

    Debugt("构建 Arm...");
    arm.reset(new Arm(&nh));

    Debugt("构建 Dataloader...");
    dataloader.reset(new Dataloader);

    Debugt("构建 Detector...");
    Detector detector;
    detector.SetDataloader(dataloader.get());
    Debugt("构建 Segmentor...");
    Segmentor segmentor;
    segmentor.SetDetector(&detector);
    Debugt("启动各个线程...");
    std::thread dct(&Detector::Run,&detector);
    std::thread sgt(&Segmentor::Run,&segmentor);
    Debugt("注册订阅器和发布器...");
    ros::Publisher pcl_pub=nh.advertise<sensor_msgs::PointCloud2>(Config::kPointCloudOutputTopic, 1);
    ros::Publisher obj_pub=nh.advertise<visualization_msgs::MarkerArray>(Config::kObjectTopic, 10);
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_detect = it.advertise(Config::kDetectResultTopic, 1);

    //接受RGB图和深度图
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, Config::kRealsense2RgbTopic, 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, Config::kRealsense2DepthTopic, 1);
    message_filters::Synchronizer<sync_pol> image_sub(sync_pol(10), rgb_sub,depth_sub);
  //  image_sub.registerCallback(boost::bind(GrabRGBD,_1,_2));
    image_sub_ptr=&image_sub;//等到获取了相机内参后再注册回调函数
    image_info_sub.reset(new ros::Subscriber(nh.subscribe(Config::kRealsense2CameraInfoTopic, 1, InfoCallback)));
   cv::namedWindow("Segmentor");
   cv::setMouseCallback("Segmentor",MouseCallback);//设置回调函数

    cout<<"loop:"<<endl;
    int counter=0;

    ros::Rate loop_rate(40);

    while(ros::ok())
    {
        ros::spinOnce();
        Frame::Ptr frame;
        if(!segmentor.WaitForFrame(frame,15)){  //获得数据
            continue;
        }
        if(frame->pc->size()>100){
            current=frame;
            cv::imshow("Segmentor",frame->drawedImg);
            cv::waitKey(1);

            sensor_msgs::PointCloud2 msg_pc;
            pcl::toROSMsg(*(frame->pc),msg_pc);
            msg_pc.header.frame_id=Config::kPointCloudMsgFrame;
            pcl_pub.publish(msg_pc);

            visualization_msgs::MarkerArray markers;
            for(auto &inst:frame->insts){
                if(!inst.pc->empty()){
                    visualization_msgs::Marker lineStripMarker,textMarker;
                    BuildLineStripMarker(inst,lineStripMarker);
                    BuildTextMarker(inst,textMarker);
                    markers.markers.push_back(lineStripMarker);
                    markers.markers.push_back(textMarker);
                }
            }
            obj_pub.publish(markers);
        }
    }

    cv::destroyAllWindows();

    cout<<"正在结束..."<<endl;
    cfg::is_running=false;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ros::shutdown();
    cout<<"程序结束"<<endl;
    return 0;
}