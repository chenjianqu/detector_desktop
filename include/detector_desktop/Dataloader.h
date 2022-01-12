/*******************************************************
 *
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of detector_desktop.
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef DETECTOR_DATALOADER_H
#define DETECTOR_DATALOADER_H

#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include "detector_desktop/Config.h"
#include "detector_desktop/Frame.h"

class Dataloader{
public:
    typedef std::shared_ptr<Dataloader> Ptr;
    Dataloader();

    void Run();
    void WaitForFrame(cv::Mat &color, cv::Mat &depth, std::chrono::steady_clock::time_point &timestamp);
    void WaitForFrame(Frame::Ptr &frame);
    void PushBack(cv::Mat color, cv::Mat depth);

private:
    std::queue<Frame::Ptr> frame_queue;
    std::mutex queue_mutex;
    std::condition_variable queue_cond;
};




#endif //DETECTOR_DATALOADER_H
