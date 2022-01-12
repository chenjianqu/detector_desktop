#include<string>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>

#include "detector_desktop/Config.h"
#include "detector_desktop/Detector.h"
#include "detector_desktop/Dataloader.h"
#include "detector_desktop/Segmentor.h"
#include "detector_desktop/Arm.h"

using namespace std;


Arm::Ptr arm;
Frame::Ptr current;

void MouseCallback(int event,int x,int y,int flags,void *p)
{
    if(event==CV_EVENT_FLAG_LBUTTON){//鼠标左键
        for(auto &inst : current->insts){
            if(inst.box.IsBelong(x,y)){//点击在这个包围框内
                Debugt("尝试抓取·{}",Box2D::class_names[inst.box.classId]);
                if(!arm->TryRunArm(inst)){
                    Errort("尝试抓取失败，机械臂正在运行...");
                }
                break;//一次只抓取一个就好了
            }
        }
    }
}


int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");//防止中文乱码
    ros::init(argc, argv, "detect_node");
    ros::start();
    ros::NodeHandle nh;

    std::string config_file;
    if(!nh.getParam("config_file",config_file)){
        config_file="/home/chen/ws/detector_ws/src/detector_desktop/config/config.yaml";
    }
    Config cfg(config_file);

    ros::Publisher pcl_pub=nh.advertise<sensor_msgs::PointCloud2>(Config::kPointCloudOutputTopic, 1);
    Infot("构建 Arm...");
    arm.reset(new Arm(&nh));
    Infot("构建 Dataloader...");
    Dataloader dataloader;
    Infot("构建 Detector...");
    Detector detector;
    detector.SetDataloader(&dataloader);
    Infot("构建 Segmentor...");
    Segmentor segmentor;
    segmentor.SetDetector(&detector);
    Infot("启动各个线程...");
    std::thread dlt(&Dataloader::Run,&dataloader);
    std::thread dct(&Detector::Run,&detector);
    std::thread sgt(&Segmentor::Run,&segmentor);
    cv::namedWindow("Detector");
    cv::setMouseCallback("Detector",MouseCallback);//设置回调函数

    int counter=0;
    sensor_msgs::PointCloud2 msg_pc;
    while(ros::ok())
    {
        Frame::Ptr frame;
        if(! segmentor.WaitForFrame(frame,50)){
            ros::spinOnce();
            continue;
        }
        current=frame;
        for(auto &inst : frame->insts){
            if(!inst.pc->empty())
                Debugt("{}:({},{},{})", inst.box.GetClassLabel(), inst.centerPt.x, inst.centerPt.y, inst.centerPt.z);
        }
        if(frame->pc->size()>100){
            pcl::toROSMsg(*(frame->pc),msg_pc);
            msg_pc.header.frame_id=Config::kPointCloudMsgFrame;
            pcl_pub.publish(msg_pc);
        }
        ros::spinOnce();
        //loop_rate.sleep();//不需要sleep，会在条件变量那里等
    }

    cfg::is_running = false;
    cv::destroyAllWindows();
    Debugt("正在结束...");
    std::this_thread::sleep_for(1s);
    ros::shutdown();
    Debugt("程序结束");
    return 0;
}