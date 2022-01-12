//
// Created by chen on 2021/8/9.
//

#include "detector_desktop/Detector.h"
#include "detector_desktop/Types.h"

std::vector<std::string> Box2D::class_names;

Detector::Detector()
{
    if(Config::use_libtorch){
        detector_base.reset(new DetectorLibtorch);
    }
    else{
#ifdef NCNN_Support
        //detectorBase.reset(new DetectorNCNN);
#else
        cerr<<"编译时未打开NCNN选项...程序将结束"<<endl;
        std::abort();
#endif
    }
    //read names
    std::ifstream f(Config::kNamesPath);
    std::string name;
    while (std::getline(f, name)){
        Box2D::class_names.push_back(name);
    }
    f.close();
}

void Detector::SetDataloader(Dataloader* dataloader_) {
    dataloader.reset(dataloader_);
}

void Detector::WaitForResult(Frame::Ptr &frame) {
    std::unique_lock<std::mutex> lock(queue_mutex);
    queue_cond.wait(lock, [&]{return !detect_queue.empty();});
    frame=detect_queue.front();
    //detectQueue.front()->clone(frame);
    detect_queue.pop();
}

void Detector::Run()
{
    TimeCounter time_counter;
    while(cfg::is_running){
        Frame::Ptr frame;
        dataloader->WaitForFrame(frame);
        Box2D::BoxesPtr result;
        detector_base->DoDetect(frame->color, result);
        frame->boxes=*result;
        //cv::putText(frame->drawedImg,cv::format("%.2f ms", timeCounter.CountMs()),cv::Point(0,30),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        {
            std::unique_lock<std::mutex> lock(queue_mutex);//使得队列长度保持
            if(detect_queue.size() < Config::kDetectorQueueSize)
                detect_queue.push(frame);
            queue_cond.notify_one();
        }
    }
    Debugt("Detector线程结束");
}


