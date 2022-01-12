/*******************************************************
 *
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of detector_desktop.
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef LIBTORCH_DETECT_DETECT_H
#define LIBTORCH_DETECT_DETECT_H

#include <vector>
#include <memory>

#include <opencv2/opencv.hpp>


#include "detector_desktop/Config.h"
#include "detector_desktop/Dataloader.h"
#include "detector_desktop/Types.h"
#include "detector_desktop/Frame.h"
#include "detector_desktop/DetectorBase.h"
#include "detector_desktop/DetectorLibtorch.h"

#define NCNN_Support
#ifdef NCNN_Support
//#include "detector_desktop/DetectorNCNN.h"
#endif


class Detector{
public:
    typedef std::shared_ptr<Detector> Ptr;
    Detector();
    void Run();
    void WaitForResult(Frame::Ptr &frame);
    void SetDataloader(Dataloader* dataloader_);
private:
    Dataloader::Ptr dataloader;
    DetectorBase::Ptr detector_base;

    std::queue<Frame::Ptr> detect_queue;
    std::mutex queue_mutex;
    std::condition_variable queue_cond;
};



#endif //LIBTORCH_DETECT_DETECT_H
