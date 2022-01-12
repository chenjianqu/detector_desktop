/*******************************************************
 *
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of detector_desktop.
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "detector_desktop/Dataloader.h"
#include <librealsense2/rs.hpp>

Dataloader::Dataloader()
{

}


void Dataloader::WaitForFrame(cv::Mat &color, cv::Mat &depth, std::chrono::steady_clock::time_point &timestamp) {
    std::unique_lock<std::mutex> lock(queue_mutex);
    queue_cond.wait(lock, [&]{return !frame_queue.empty();});
    color=frame_queue.front()->color;
    depth=frame_queue.front()->depth;
    timestamp=frame_queue.front()->timestamp;
    frame_queue.pop();
}

void Dataloader::WaitForFrame(Frame::Ptr &frame) {
    std::unique_lock<std::mutex> lock(queue_mutex);
    queue_cond.wait(lock, [&]{return !frame_queue.empty();});
    frame=frame_queue.front();
    //frame_queue.front()->clone(frame);
    frame_queue.pop();
}


void Dataloader::PushBack(cv::Mat color, cv::Mat depth)
{
    std::unique_lock<std::mutex> lock(queue_mutex);
    if(frame_queue.size() < cfg::kDataloaderQueueSize)
        frame_queue.emplace(new Frame(color, depth, std::chrono::steady_clock::now()));
    queue_cond.notify_one();
}


void Dataloader::Run() {
    Debugt("正在初始化相机...");
    //用于深度图像和color图像的对齐
    rs2::align depth_to_color_aligner(RS2_STREAM_COLOR);
    //用于剔除深度值
    rs2::threshold_filter depth_threshold_filter;
    depth_threshold_filter.set_option(RS2_OPTION_MIN_DISTANCE, cfg::kDepthFilterThresholdMin);
    depth_threshold_filter.set_option(RS2_OPTION_MAX_DISTANCE, cfg::kDepthFilterThresholdMax);
    //相机的配置
    rs2::config rscfg;
    rscfg.enable_stream(RS2_STREAM_COLOR, cfg::kCameraWidth, cfg::kCameraHeight,
                        RS2_FORMAT_BGR8, cfg::kCameraFps);
    rscfg.enable_stream(RS2_STREAM_DEPTH,RS2_FORMAT_Z16,cfg::kCameraFps);//因为后面要对齐，所以深度数据流的大小为默认
    //启动pipeline
    rs2::pipeline pipe;
    pipe.start(rscfg);

    //获得相机的重要参数，并输出
    auto frames=pipe.wait_for_frames();
    frames=depth_to_color_aligner.process(frames);//对齐

    const int color_width=frames.get_color_frame().get_width();
    const int color_height=frames.get_color_frame().get_height();
    const int depth_width=frames.get_depth_frame().get_width();
    const int depth_height=frames.get_depth_frame().get_height();

    Debugt("Realsense2的Color大小：{} {}",color_width,color_height);
    Debugt("Realsense2的Depth大小：{} {}",depth_width,depth_height);
    if (frames.get_color_frame().get_profile().format() == RS2_FORMAT_BGR8)
        Debugt("Realsense2的Color数据格式：BGR8");
    else if (frames.get_color_frame().get_profile().format() == RS2_FORMAT_RGB8)
        Debugt("Realsense2的Color数据格式：RGB8");
    else
        Debugt("Realsense2的Color数据格式：{}",frames.get_color_frame().get_profile().format());

    //开始采集数据，并放入队列中
    while(cfg::is_running){
        frames=pipe.wait_for_frames();//获得数据
        auto time_now=std::chrono::steady_clock::now();
        frames=depth_to_color_aligner.process(frames);//对齐
        cv::Mat color(cv::Size(color_width,color_height),CV_8UC3,(void*)frames.get_color_frame().get_data());
        cv::Mat depth(cv::Size(depth_width,depth_height),CV_16UC1,(void*)frames.get_depth_frame().get_data());
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            if(frame_queue.size() < cfg::kDataloaderQueueSize) //使得队列长度保持<=10
                frame_queue.emplace(new Frame(color, depth, time_now));
            queue_cond.notify_one();
        }
    }
    Debugt("Dataloader线程结束");



}



