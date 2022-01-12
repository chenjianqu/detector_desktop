/*******************************************************
 *
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of detector_desktop.
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef LIBTORCH_DETECT_CONFIG_H
#define LIBTORCH_DETECT_CONFIG_H

#include <string>
#include <memory>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <spdlog/logger.h>
#include <spdlog/spdlog.h>

using namespace std;

class Config{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Config> Ptr;
    explicit Config(const string &file_name);

    inline static string kRealsense2RgbTopic;
    inline static string kRealsense2DepthTopic;
    inline static string kRealsense2CameraInfoTopic;

    inline static string kPointCloudOutputTopic;
    inline static string kPointCloudMsgFrame;

    inline static string kNamesPath;
    inline static string kModelPath;

    inline static float kCX,kCY,kFX,kFY;
    inline static float kDepthFactor;

    inline static float kDepthFilterThresholdMin,kDepthFilterThresholdMax;
    inline static int kCameraWidth,kCameraHeight;
    inline static int kCameraFps;

    inline static string kKinovaRobotType;
    inline static string kKinovaPoseActionAddress;
    inline static string kArmMsgFrame;
    inline static string kKinovaFingerActionAddress;
    inline static float kArmFingerValue;

    inline static cv::Mat kOpticalToBase;
    inline static cv::Mat kCamToArm;
    inline static cv::Mat kOpticalToCam;
    inline static cv::Mat kGraspPoseEuler;
    inline static cv::Mat kOpticalToArm;

    inline static int kDetectorQueueSize,kDataloaderQueueSize,kSegmentorQueueSize;

    inline static bool use_libtorch;
    inline  static bool use_gpu;

    inline static int kNcnnNumThreads;

    inline static float kPointCloudResolution;
    inline static float kObjectThreshold;

    inline static string kObjectTopic;
    inline static string kDetectResultTopic;

    inline static  std::shared_ptr<spdlog::logger> logger;

    inline static std::atomic<bool> is_running=true;
};

using cfg = Config;

template <typename Arg1, typename... Args>
inline void Debugt(const char* fmt, const Arg1 &arg1, const Args&... args){ cfg::logger->log(spdlog::level::debug, fmt, arg1, args...);}
template<typename T>
inline void Debugt(const T& msg){cfg::logger->log(spdlog::level::debug, msg); }
template <typename Arg1, typename... Args>
inline void Infot(const char* fmt, const Arg1 &arg1, const Args&... args){cfg::logger->log(spdlog::level::info, fmt, arg1, args...);}
template<typename T>
inline void Infot(const T& msg){cfg::logger->log(spdlog::level::info, msg);}
template <typename Arg1, typename... Args>
inline void Warnt(const char* fmt, const Arg1 &arg1, const Args&... args){cfg::logger->log(spdlog::level::warn, fmt, arg1, args...);}
template<typename T>
inline void Warnt(const T& msg){cfg::logger->log(spdlog::level::warn, msg);}
template <typename Arg1, typename... Args>
inline void Errort(const char* fmt, const Arg1 &arg1, const Args&... args){cfg::logger->log(spdlog::level::err, fmt, arg1, args...);}
template<typename T>
inline void Errort(const T& msg){cfg::logger->log(spdlog::level::err, msg);}
template <typename Arg1, typename... Args>
inline void Criticalt(const char* fmt, const Arg1 &arg1, const Args&... args){cfg::logger->log(spdlog::level::critical, fmt, arg1, args...);}
template<typename T>
inline void Criticalt(const T& msg){cfg::logger->log(spdlog::level::critical, msg);}


#endif //LIBTORCH_DETECT_CONFIG_H
