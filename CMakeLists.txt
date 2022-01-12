cmake_minimum_required(VERSION 3.0.2)
project(detector_desktop)

add_compile_options(-std=c++17)

option(NCNN_Support "Enable NCNN_Support to Detection" Off)

set(NCNN_PATH /home/chen/app/ncnn)
set(Libtorch_PATH /home/chen/app/libtorch)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cv_bridge
  image_transport
  message_filters
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  image_geometry
)

catkin_package(
)

find_package(spdlog REQUIRED)
include_directories(${spdlog_INCLUDE_DIRS})

find_package(OpenCV 3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(realsense2 2 REQUIRED)
find_package(Eigen3 3 REQUIRED)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_LIBS}
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${realsense2_INCLUDE_DIR}
        /home/chen/ws/robot_ws/devel/include
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Libtorch相关
list(APPEND CMAKE_PREFIX_PATH ${Libtorch_PATH})
find_package(Torch REQUIRED)
find_package(TorchVision REQUIRED)
include_directories(${TORCH_INCLUDE_DIRS})
set(LIBS_Libtorch  ${TORCH_LIBRARIES}  TorchVision::TorchVision)


# NCNN相关
if(NCNN_Support)
    find_package(Vulkan)
    include_directories(
            ${NCNN_PATH}/src
            ${NCNN_PATH}/build/src
            ${NCNN_PATH}/glslang/OGLCompilersDLL
            ${NCNN_PATH}/glslang/SPIRV
            ${NCNN_PATH}/glslang/glslang/glslang/OSDependent
            ${NCNN_PATH}/glslang/glslang/glslang/Public
    )
    set(LIBS_NCNN
            ${NCNN_PATH}/build/src/libncnn.a #这个一定要放在前面

            ${NCNN_PATH}/build/glslang/glslang/libglslang.a
            ${NCNN_PATH}/build/glslang/glslang/libMachineIndependent.a
            ${NCNN_PATH}/build/glslang/glslang/libGenericCodeGen.a
            ${NCNN_PATH}/build/glslang/OGLCompilersDLL/libOGLCompiler.a
            ${NCNN_PATH}/build/glslang/SPIRV/libSPIRV.a
            ${NCNN_PATH}/build/glslang/glslang/OSDependent/Unix/libOSDependent.a

            -lpthread #多线程库
            -fopenmp #对于GCC来说，启用openmp编译
            -lvulkan #连接libvulkan
            -lgcc_s #因为没加这个，bug困扰了我一整天
            -lgcc)
endif()


add_library(detector_desktop_config include/detector_desktop/Config.h src/Config.cpp)
target_link_libraries(detector_desktop_config ${catkin_LIBRARIES} ${Libs})
add_dependencies(detector_desktop_config ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


if(NCNN_Support)
    add_library(detector_desktop_DetectorBase include/detector_desktop/DetectorBase.h
            include/detector_desktop/DetectorNCNN.h src/DetectorNCNN.cpp
            include/detector_desktop/DetectorLibtorch.h src/DetectorLibtorch.cpp)
    target_link_libraries(detector_desktop_DetectorBase ${catkin_LIBRARIES} ${Libs} ${LIBS_Libtorch} ${LIBS_NCNN})
    add_dependencies(detector_desktop_DetectorBase ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
else()
    add_library(detector_desktop_DetectorBase include/detector_desktop/DetectorBase.h
            include/detector_desktop/DetectorLibtorch.h src/DetectorLibtorch.cpp)
    target_link_libraries(detector_desktop_DetectorBase ${catkin_LIBRARIES} ${Libs} ${LIBS_Libtorch})
    add_dependencies(detector_desktop_DetectorBase ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()


set(MyLib detector_desktop_config  detector_desktop_DetectorBase)


add_executable(detector_desktop src/main.cpp src/Segmentor.cpp src/Detector.cpp  src/Dataloader.cpp src/Arm.cpp src/utils.cpp)
target_link_libraries(detector_desktop ${catkin_LIBRARIES}
        ${spdlog_LIBRARIES}  ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${realsense2_LIBRARY} ${MyLib})

add_executable(detector_desktop_ros src/main_ros.cpp src/Detector.cpp src/Segmentor.cpp src/Dataloader.cpp src/Arm.cpp src/utils.cpp)
target_link_libraries(detector_desktop_ros ${catkin_LIBRARIES}
        ${spdlog_LIBRARIES}  ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${realsense2_LIBRARY} ${MyLib})

add_executable(detector_desktop_ros_pcl_test src/ros_pcl_test.cpp)
target_link_libraries(detector_desktop_ros_pcl_test ${catkin_LIBRARIES}  ${MyLib}
        ${spdlog_LIBRARIES}  ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${realsense2_LIBRARY})
