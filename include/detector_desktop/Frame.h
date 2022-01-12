/*******************************************************
 *
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of detector_desktop.
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef DETECTOR_FRAME_H
#define DETECTOR_FRAME_H

#include <opencv2/opencv.hpp>
#include <condition_variable>
#include <chrono>

#include "detector_desktop/Types.h"

class Frame{
public:
    typedef std::shared_ptr<Frame> Ptr;

    Frame()=default;

    Frame(cv::Mat &color_,cv::Mat &depth_,TimeType timestamp_)
        :color(color_),depth(depth_),timestamp(timestamp_),pc(new PointCloud)
    {
        drawedImg=color.clone();
    }
    //移动构造函数
    Frame(cv::Mat &&color_,cv::Mat &&depth_,TimeType &&timestamp_,std::vector<Box2D> &&boxes_,std::vector<Instance> &&insts_,cv::Mat &&drawedImg_)
        :color(color_),depth(depth_),timestamp(timestamp_),boxes(boxes_),insts(insts_),drawedImg(drawedImg_)
    {
    }

    void clone(Frame::Ptr &frame){//深度拷贝函数，代价很大
        frame.reset(new Frame);
        frame->color=this->color.clone();
        frame->depth=this->depth.clone();
        frame->drawedImg=this->drawedImg.clone();
        frame->boxes=this->boxes;
        frame->insts=this->insts;
        frame->pc=this->pc;//点云未实现深度拷贝
    }

    void MergePointCloud(){
        pc->clear();
        for(auto &inst : insts) if(!inst.pc->empty()) *pc+=*(inst.pc);
    }

    void VisBoxes(cv::Scalar &color){
        for(auto &box : boxes) box.VisBox(drawedImg,color);
    }


    cv::Mat color,depth;
    cv::Mat drawedImg;
    TimeType timestamp;

    std::vector<Box2D> boxes;
    std::vector<Instance> insts;
    PointCloud::Ptr pc;
private:

};


class DetectFrame : public Frame{

};





#endif //DETECTOR_FRAME_H
