/*******************************************************
 *
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of detector_desktop.
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "detector_desktop/Config.h"
#include <filesystem>

namespace fs=std::filesystem;


void InitLogger()
{
    auto resetLogFile=[](const std::string &path){
        if(!fs::exists(path)){
            std::ifstream file(path);//创建文件
            file.close();
        }
        else{
            std::ofstream file(path,std::ios::trunc);//清空文件
            file.close();
        }
    };

    auto getLogLevel=[](const std::string &level_str){
        if(level_str=="debug")
            return spdlog::level::debug;
        else if(level_str=="info")
            return spdlog::level::info;
        else if(level_str=="warn")
            return spdlog::level::warn;
        else if(level_str=="error" || level_str=="err")
            return spdlog::level::err;
        else if(level_str=="critical")
            return spdlog::level::critical;
        else{
            cerr<<"log level not right, set default warn"<<endl;
            return spdlog::level::warn;
        }
    };

    resetLogFile("log.txt");
    cfg::logger = spdlog::basic_logger_mt("log","log.txt");
    cfg::logger->set_level(getLogLevel("debug"));
    cfg::logger->flush_on(getLogLevel("debug"));
}

Config::Config(const string &file_name)
{
    cv::FileStorage fs(file_name,cv::FileStorage::READ);
    fs["realsense2_rgb_topic"] >> kRealsense2RgbTopic;
    fs["realsense2_depth_topic"] >> kRealsense2DepthTopic;
    fs["realsense2_camera_info_topic"] >> kRealsense2CameraInfoTopic;
    fs["point_cloud_output_topic"] >> kPointCloudOutputTopic;
    fs["point_cloud_msg_frame"] >> kPointCloudMsgFrame;

    fs["model_path"] >> kModelPath;
    fs["names_path"] >> kNamesPath;

    fs["kinova_robotType"] >> kKinovaRobotType;
    fs["kinova_pose_action_address"] >> kKinovaPoseActionAddress;
    fs["arm_msg_frame"] >> kArmMsgFrame;
    fs["kinova_finger_action_address"] >> kKinovaFingerActionAddress;
    fs["arm_finger_value"] >> kArmFingerValue;

    fs["optical2base"] >> kOpticalToBase;
    fs["cam2arm"] >> kCamToArm;
    fs["optical2cam"] >> kOpticalToCam;
    fs["grasp_pose_euler"] >> kGraspPoseEuler;
    fs["optical2arm"] >> kOpticalToArm;

    fs["Camera.cx"] >> kCX;
    fs["Camera.cy"] >> kCY;
    fs["Camera.fx"] >> kFX;
    fs["Camera.fy"] >> kFY;
    fs["Camera.depthFactor"] >> kDepthFactor;
    fs["Camera.width"] >> kCameraWidth;
    fs["Camera.height"] >> kCameraHeight;
    fs["Camera.fps"] >> kCameraFps;

    fs["depth_filter_threshold_min"] >> kDepthFilterThresholdMin;
    fs["depth_filter_threshold_max"] >> kDepthFilterThresholdMax;

    fs["detector_queue_size"] >> kDetectorQueueSize;
    fs["segmentor_queue_size"] >> kSegmentorQueueSize;
    fs["dataloader_queue_size"] >> kDataloaderQueueSize;

    fs["use_libtorch"]>>use_libtorch;
    fs["use_gpu"]>>use_gpu;
    fs["ncnn_num_threads"] >> kNcnnNumThreads;

    fs["point_cloud_resolution"] >> kPointCloudResolution;

    fs["object_threshold"] >> kObjectThreshold;

    fs["object_topic"] >> kObjectTopic;
    fs["detect_result_topic"] >> kDetectResultTopic;
    fs.release();

    InitLogger();

    cout<<"参数初始化完毕\n\n"<<endl;
}




