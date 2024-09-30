/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef SYSTEM_H
#define SYSTEM_H


#include <unistd.h>
#include<stdio.h>
#include<stdlib.h>
#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"
#include "ImuTypes.h"
#include "Settings.h"


namespace ORB_SLAM3
{

class Verbose
{
public:
    enum eLevel
    {
        VERBOSITY_QUIET=0,
        VERBOSITY_NORMAL=1,
        VERBOSITY_VERBOSE=2,
        VERBOSITY_VERY_VERBOSE=3,
        VERBOSITY_DEBUG=4
    };

    static eLevel th;

public:
    static void PrintMess(std::string str, eLevel lev)
    {
        if(lev <= th)
            cout << str << endl;
    }

    static void SetTh(eLevel _th)
    {
        th = _th;
    }
};

class Viewer;
class FrameDrawer;
class MapDrawer;
class Atlas;
class Tracking;
class LocalMapping;
class LoopClosing;
class Settings;

class System
{
public:
    // Input sensor,输入传感器的类型
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2,
        IMU_MONOCULAR=3,
        IMU_STEREO=4,
        IMU_RGBD=5,
    };

    // File type
    enum FileType{
        TEXT_FILE=0,
        BINARY_FILE=1,
    };

public:
    // 内存对齐: Eigen 库中的许多类型,如 Eigen::Vector3d 和 Eigen::Matrix4f等,需要内存对齐以提高性能。 EIGEN_MAKE_ALIGNED_OPERATOR_NEW 宏确保了这些类型在内存中被正确地对齐。
    // 自定义内存分配: 这个宏会定义自定义的 new 和 delete 操作符,使用 Eigen 库的内存分配器来管理内存。这样可以确保 Eigen 对象被正确地分配和释放内存
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    // '''
    // Local Mapping:
    //     *新关键帧的处理:当一个新的关键帧被添加到系统中时,Local Mapping 模块会对其进行处理。它会检查这个新关键帧是否与当前地图有足够的重叠特征点,从而可以与现有地图进行融合。
    //     *局部地图的构建和优化:Local Mapping 模块会构建一个局部地图,包括当前关键帧及其邻近的关键帧。它会使用 BA (Bundle Adjustment) 算法优化这个局部地图,以提高地图的一致性和精度。
    //     *新地图点的创建:对于新的关键帧,Local Mapping 会尝试创建新的地图点。它会利用关键帧之间的重叠特征点,三角化出新的 3D 地图点。
    //     *地图点的优化和剔除:Local Mapping 会对地图点进行优化,以提高它们的位置估计。同时,它也会剔除那些观测数量太少或位置不确定的地图点。
    //     *全局地图的更新:当局部地图优化完成后,Local Mapping 会将更新后的关键帧和地图点信息传递给全局地图管理模块。这样可以确保全局地图保持最新和一致。
    //     *线程管理:Local Mapping 模块是一个独立的线程,与 Tracking 和 Loop Closing 模块并行运行。它通过消息队列和互斥锁来与其他模块进行通信和同步。
    
    // Tracking
    //     *初始化:Tracking 线程在系统启动时会进行初始化,包括加载词典、初始化地图等。它还会启动其他的模块,如 Local Mapping 和 Loop Closing 线程。
    //     *实时跟踪:Tracking 线程会实时处理来自相机的图像帧,并跟踪相机的位姿变化。它会尝试将当前帧中的特征点与地图中的特征点进行匹配,并根据匹配结果计算相机的位姿。
    //     *关键帧的选取:Tracking 线程会根据一定的标准,选择部分帧作为关键帧保存到地图中。关键帧的选取会考虑诸如视角变化、特征点数量等因素。
    //     *Local BA 优化:对于新添加的关键帧,Tracking 线程会触发 Local Mapping 模块进行局部 BA 优化,以提高地图的一致性。
    //     *回环检测:Tracking 线程会将当前帧的特征点描述子发送给 Loop Closing 线程,以检测是否存在回环。如果检测到回环,Tracking 线程会暂时冻结,等待 Loop Closing 线程完成回环优化。
    //     *地图更新: Tracking 线程会将新的关键帧和地图点信息更新到全局地图中。它还会从 Local Mapping 和 Loop Closing 线程接收更新,并将其应用到当前的跟踪状态中。
    //     *线程管理:Tracking 线程作为 ORB-SLAM3 系统的主线程,负责协调其他模块的工作。它通过消息队列和互斥锁来与 Local Mapping 和 Loop Closing 线程进行通信和同步。
    
    // Loop Closing:
    //     *循环闭合检测:Loop Closing 线程会定期从 Tracking 线程接收当前帧的特征点描述子,并与地图中已有的关键帧进行匹配,检测是否存在回环。它会使用 DBoW2 (Discrete Bayes Words) 库进行高效的场景匹配,以快速检测潜在的回环。
    //     *回环验证:对于检测到的潜在回环,Loop Closing 会进一步验证是否真的存在回环。它会使用几何一致性检查、投影一致性检查等方法对匹配结果进行验证,确保回环检测的可靠性。
    //     *位姿图优化:当确认存在回环时,Loop Closing 会触发全局优化,对位姿图进行优化。它使用 g2o (General Graph Optimization) 库实现基于图优化的方法,同时考虑回环约束和其他约束,以最小化整个系统的位姿误差。
    //     *地图更新:优化位姿图之后,Loop Closing 会将更新后的关键帧和地图点信息反馈给 Tracking 和 Local Mapping 线程,以确保地图保持一致。它还会修正在回环检测之前产生的错误,消除累积误差。
    //     *线程管理: Loop Closing 线程作为一个独立的线程,与 Tracking 和 Local Mapping 线程并行运行。它通过消息队列和互斥锁来与其他线程进行通信和同步,确保各模块之间的协调工作。

    // Viewer threads:
    //     可视化地图:Viewer 线程会实时渲染并显示当前建立的地图,包括关键帧、地图点等。它可以使用 OpenGL 或 PCL (Point Cloud Library) 等库进行三维场景的渲染和可视化。
    //     显示相机轨迹:Viewer 线程会绘制相机的实时轨迹,帮助用户了解相机的运动状态。轨迹可以用线条或其他形状进行可视化表示。
    //     显示调试信息:Viewer 线程可以在可视化窗口中显示一些调试信息,如当前帧数、跟踪状态、地图大小等。这些信息有助于用户监控 SLAM 系统的运行状况。
    //     交互功能:Viewer 线程可以提供一些交互功能,如允许用户缩放、旋转和平移地图视图等。这些交互功能可以帮助用户更好地分析和理解 SLAM 系统的内部状态。
    //     多线程协作:Viewer 线程是一个独立的线程,与 Tracking、Local Mapping 和 Loop Closing 等线程并行运行。它通过消息队列和互斥锁与其他线程进行通信,获取最新的地图和相机状态信息。
    //     可选性:Viewer 线程是 ORB-SLAM3 系统的可选组件,用户可以选择是否启用它。在一些资源受限的环境中,用户可以选择禁用 Viewer 线程,以提高 SLAM 系统的整体性能。
    // '''

    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true, const int initFr = 0, const string &strSequence = std::string());

    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).

    // SE(3) 表示 3D 空间中的刚体变换,包括旋转和平移。这种变换可以用 4x4 的齐次矩阵来表示
    Sophus::SE3f TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    Sophus::SE3f TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");

    // Proccess the given monocular frame and optionally imu data
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    Sophus::SE3f TrackMonocular(const cv::Mat &im, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");


    // This stops local mapping thread (map building) and performs only camera tracking.
    // 该函数用于停止本地建图线程,仅保留相机跟踪功能。
    // 在某些情况下,比如当系统已有一个稳定的地图,用户只需要定位当前相机位置时,可以选择激活定位模式。
    // 这样可以减轻系统开销,提高定位的实时性
    void ActivateLocalizationMode();
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    // 这个函数在 Viewer 线程中很有用,因为它可以帮助 Viewer 线程判断何时需要重新渲染和更新地图的可视化表示。具体的工作原理如下:
    // 在 Tracking 线程、Local Mapping 线程和 Loop Closing 线程中,当地图发生变化时(比如增加了新的关键帧或地图点),它们会设置一个标志位,表示地图发生了变化。
    // Viewer 线程会周期性地调用 MapChanged() 函数来检查地图是否发生变化。如果函数返回 true，说明地图有更新,Viewer 线程就会重新渲染和更新地图的可视化效果。 
    // 在地图变化的标志位被设置后,MapChanged() 函数会返回 true。一旦 Viewer 线程完成了地图的重新渲染,它会重置这个标志位,使得下次调用 MapChanged() 函数时返回 false。
    bool MapChanged();

    // Reset the system (clear Atlas or the active map)
    // Reset()函数:
    // 这个函数用于完全重置整个 ORB-SLAM3 系统,包括清空 Atlas(全局地图)和所有的活跃地图。调用这个函数后,系统会回到初始状态,所有之前构建的地图信息都会被清除。
    // 这通常在需要重新启动或重新初始化整个 SLAM 系统时使用,例如在更换环境或传感器时。
    // ResetActiveMap()函数:这个函数仅用于重置当前激活的地图,而不会影响 Atlas 或其他地图。调用这个函数后,系统会清空当前正在使用的地图,但不会影响其他地图的状态。
    //这在某些情况下很有用,比如当前地图出现问题或需要重新初始化时,可以单独重置当前地图,而不需要重置整个系统。
    void Reset();
    void ResetActiveMap();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    // Shutdown()函数:
    // 这个函数用于关闭和终止 ORB-SLAM3 系统的运行。当调用这个函数时，系统会执行以下操作:停止所有正在运行的线程(Tracking、LocalMapping、LoopClosing等)。
    // 保存当前构建的地图信息。 释放系统中使用的所有资源,如内存、数据结构等。 执行完这些操作后,系统就进入了关闭状态,不再接受任何新的输入或处理。
    // isShutDown()函数:
    // 这个函数用于检查系统当前是否处于关闭状态。 它返回一个布尔值,true表示系统已经关闭,false表示系统仍在运行。这个函数通常用于在其他线程或模块中判断系统是否还在运行,从而决定是否继续执行相关操作。
    void Shutdown();
    bool isShutDown();

    // Save camera trajectory in the TUM RGB-D dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    // 将当前的轨迹保存为 TUM 格式文件的函数。timestamp tx ty tz qx qy qz qw 
    // timestamp: 时间戳（单位：秒）
    // tx, ty, tz: 相机位置的 x、y、z 坐标
    // qx, qy, qz, qw: 相机姿态的四元数表示
    void SaveTrajectoryTUM(const string &filename);

    // Save keyframe poses in the TUM RGB-D dataset format.
    // This method works for all sensor input.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    // 保存关键帧的轨迹
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    // EuRoC格式 #timestamp [ns] q_w q_x q_y q_z t_x t_y t_z
    void SaveTrajectoryEuRoC(const string &filename);
    void SaveKeyFrameTrajectoryEuRoC(const string &filename);

    // 这个函数的主要作用是将当前系统中记录的相机轨迹数据和构建的地图信息一起保存到指定的 EuRoC 格式文件中,以供后续分析和评估使用。
    // #timestamp [ns] q_w q_x q_y q_z t_x t_y t_z
    // 1403636570642190000 1.0 0.0 0.0 0.0 0.0 0.0 0.0
    // 1403636570652190000 0.999 0.002 0.045 -0.014 0.001 0.002 0.003
    // 1403636570662190000 0.998 0.001 0.055 -0.007 0.002 0.004 0.003
    // #landmarks
    // id_1 x_1 y_1 z_1
    // id_2 x_2 y_2 z_2

    void SaveTrajectoryEuRoC(const string &filename, Map* pMap);
    void SaveKeyFrameTrajectoryEuRoC(const string &filename, Map* pMap);

    // Save data used for initialization debug
    // '''
    // 这个函数的主要作用是将系统内部在某一特定帧（由 iniIdx 参数指定）上的一些调试信息保存到磁盘文件中,以便于开发人员进行问题分析和系统调试。
    // 保存的调试数据通常包括以下内容:
    // 当前帧的相机位姿信息
    // 当前帧的特征点检测和描述符信息
    // 当前帧的地图点投影和关联情况
    // 当前帧的视觉词袋模型匹配信息
    // 当前帧的几何一致性检查结果
    // 当前帧的视觉里程计和全局优化结果
    // '''
    void SaveDebugData(const int &iniIdx);

    // Save camera trajectory in the KITTI dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    void SaveTrajectoryKITTI(const string &filename);

    // TODO: Save/Load functions
    // SaveMap(const string &filename);
    // LoadMap(const string &filename);

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    // '''
    // GetTrackingState() 是 ORB-SLAM3 系统中用于获取当前跟踪状态的函数。
    // 该函数的主要作用是返回当前系统的跟踪状态,即相机是否成功地进行了位姿估计和地图构建。
    // 函数返回值有以下几种可能:
    // SYSTEM_NOT_READY: 系统尚未初始化完成，无法进行跟踪。
    // NO_IMAGES_YET: 系统已初始化但尚未接收到任何图像输入。
    // NOT_INITIALIZED: 系统已接收到图像输入但尚未完成初始化。
    // OK: 系统已成功完成初始化并处于正常跟踪状态。
    // LOST: 系统已经丢失跟踪，无法继续估计相机位姿。
    // 根据不同的跟踪状态,ORB-SLAM3 系统会采取不同的处理措施:

    // 当系统处于 SYSTEM_NOT_READY 或 NO_IMAGES_YET 状态时，系统将等待直到接收到足够的输入数据才能开始初始化。
    // 当系统处于 NOT_INITIALIZED 状态时，系统将尝试执行初始化过程以构建地图并确定相机位姿。
    // 当系统处于 OK 状态时，系统将继续进行实时跟踪和地图优化。
    // 当系统处于 LOST 状态时，系统将尝试通过重定位或初始化过程来重新获得跟踪。
    // GetTrackingState() 函数通常被其他模块调用以获取当前系统的跟踪状态信息。例如,用户界面可以根据跟踪状态信息来显示相应的UI提示信息。开发人员也可以使用该函数来监控系统的运行状态,并进行相应的错误处理或性能优化。
    // '''
    int GetTrackingState();


    // '''
    // GetTrackedMapPoints()
    // 这个函数返回一个 std::vector<MapPoint*> 类型的容器,其中包含了当前帧成功跟踪到的所有地图点。
    // 地图点是 ORB-SLAM3 系统中表示3D环境的基本单元,每个地图点都有其三维坐标和描述子信息。
    // 通过这个函数,我们可以获取当前帧在地图中成功匹配和定位的所有3D地图点。这些地图点信息可以用于进一步的地图维护、渲染和分析等操作。
    // GetTrackedKeyPointsUn()
    // 这个函数返回一个 std::vector<cv::KeyPoint> 类型的容器,其中包含了当前帧成功跟踪到的所有关键点坐标。
    // 关键点是在图像中检测到的具有显著特征的像素点,ORB-SLAM3 系统会提取和描述这些关键点,并用于相机位姿估计和地图构建。
    // 通过这个函数,我们可以获取当前帧所有被成功跟踪的2D关键点坐标信息。这些关键点信息可以用于绘制图像特征、计算视觉里程计等操作。
    // '''
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

    // For debugging
    // 在 ORB-SLAM3 的初始化过程中,系统需要对IMU传感器进行校准和初始化,确保IMU数据与视觉数据能够很好地融合。这个过程称为IMU初始化。
    // GetTimeFromIMUInit() 函数返回的就是从IMU初始化完成到当前时刻经过的时间,单位为秒。这个时间差信息对于以下几个方面很重要:
    // 时间同步：IMU数据和视觉数据需要进行时间同步,以确保它们能够正确地融合。GetTimeFromIMUInit() 提供的时间差信息可以用于时间对齐。
    double GetTimeFromIMUInit();

    // 这个函数返回一个布尔值,表示系统是否已经丢失跟踪。该函数的返回值可以用于触发系统的重定位或者回环检测等机制,帮助系统重新获得跟踪
    bool isLost();
    // isFinished() 这个函数返回一个布尔值,表示系统是否已经完成了所有的操作,可以安全退出了
    bool isFinished();

    // '''
    // ChangeDataset() 是 ORB-SLAM3 系统中用于切换数据集的函数。
    // 在 ORB-SLAM3 中,系统可以处理不同类型的输入数据,例如来自摄像头的视频流、来自传感器的图像序列,或者来自标准数据集的图像/点云等。
    // ChangeDataset() 函数的作用就是允许开发者在不同的数据集之间进行切换。这个功能非常有用,因为它允许开发者:
    // 测试和评估系统在不同场景和环境下的表现。可以使用不同的数据集来验证算法的鲁棒性和通用性。
    // 重复利用现有的系统配置和参数。当切换数据集时,开发者无需重新配置系统,可以直接使用之前调试好的参数。
    // 快速验证算法改动的效果。通过切换数据集,开发者可以快速地在不同场景下测试算法的变更,而无需重新采集数据。
    // 支持离线处理和批处理。通过切换数据集,开发者可以在离线环境下对整个数据集进行批量处理和分析。
    // '''
    void ChangeDataset();

    // '''
    // 在 ORB-SLAM3 中,输入图像会经过一系列预处理步骤,包括缩放、灰度转换等。其中,图像缩放是一个非常重要的步骤,因为它会影响到ORB特征点的检测和描述,进而影响整个视觉SLAM系统的性能。
    // GetImageScale() 函数返回的是当前图像相对于原始图像的缩放比例,这个比例通常是一个小于1的浮点数。这个信息在 ORB-SLAM3 系统中有以下几个重要用途:
    // 特征点处理:在提取ORB特征点时,需要根据图像缩放比例对特征点的坐标进行相应的缩放和转换,以确保特征点能够正确地对应到原始图像上。
    // 位姿估计:在相机位姿估计过程中,需要将特征点坐标从缩放后的图像空间转换到原始图像空间,以确保位姿估计的准确性。GetImageScale() 提供的缩放比例信息就是用于这个转换过程的。
    // 地图管理:在地图构建和优化过程中,也需要利用图像缩放比例来规范化地图点的坐标,以确保地图的尺度一致性。
    // 输出可视化:在将结果输出可视化时,也需要根据图像缩放比例来调整绘制元素的大小和位置,以确保输出结果能够正确地与原始图像对应。
    // '''
    float GetImageScale();

#ifdef REGISTER_TIMES
    void InsertRectTime(double& time);
    void InsertResizeTime(double& time);
    void InsertTrackTime(double& time);
#endif

private:
    // 这个函数用于将当前构建的地图数据保存到磁盘上。
    void SaveAtlas(int type);

    // 这个函数用于从磁盘加载之前保存的地图数据。
    // 参数 type 指定了要加载的地图数据类型,可以是 BINARY_FILE 或 TEXT_FILE。
    // 该函数根据指定的类型从磁盘文件中读取地图数据,并将其加载到 ORB-SLAM3 系统中。
    // 加载成功后,系统可以继续执行后续的跟踪、优化、回环检测等操作。
    // 这个函数可用于恢复之前保存的地图状态,避免重新构建整个地图。
    bool LoadAtlas(int type);

    // CalculateCheckSum() 是 ORB-SLAM3 系统中用于计算文件的校验和的函数。
    string CalculateCheckSum(string filename, int type);

    // Input sensor
    eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    // '''
    // ORBVocabulary 是一个用于存储和管理ORB特征词典的类。在 ORB-SLAM3 系统中,它起到了以下关键作用:
    // 特征点描述子的量化:
    // ORB-SLAM3 使用 ORB 特征点作为视觉特征,每个特征点都有一个 256 bit 的描述子向量。
    // 通过 ORBVocabulary 中预先训练好的视觉词典,可以将这些高维描述子向量量化为一个离散的词ID。
    // 这个量化过程大大减少了特征点描述子的存储空间,同时也便于后续的特征匹配和关键帧检测。
    // 快速特征点匹配:
    // 在进行相机位姿估计时,需要对当前帧的ORB特征点与地图中的特征点进行匹配。
    // ORBVocabulary 提供了高效的基于词袋模型的特征点匹配算法,大大加速了这个匹配过程。
    // 回环检测:
    // 当机器人在一个已经探索过的区域重新经过时,需要进行回环检测,以更正累积的位姿drift。
    // ORBVocabulary 存储的视觉词典可以用于快速检测回环,提高回环检测的准确性和鲁棒性。
    // '''
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    // '''
    // KeyFrameDatabase 是 ORB-SLAM3 中用于管理关键帧数据的类。它的主要功能包括:
    // 关键帧的存储和索引:
    // 在SLAM过程中,系统会选择一些特征信息丰富的帧作为关键帧,保存在数据库中。
    // KeyFrameDatabase 使用高效的索引结构(如词袋模型)来存储和管理这些关键帧数据。
    // 回环检测:
    // 当机器人进入一个已经探索过的区域时,需要检测是否发生了回环。
    // KeyFrameDatabase 提供了基于关键帧的回环检测算法,可以快速找到与当前帧相似的关键帧,从而检测回环。
    // 回环检测的结果可用于优化机器人的轨迹,消除累积的位姿误差。
    // 重定位:
    // 当机器人失去定位时,需要进行重定位操作,将当前观测与地图中的特征点进行匹配。
    // KeyFrameDatabase 可以快速找到与当前帧相似的关键帧,并利用它们提供的特征点信息来帮助机器人重新定位。
    // '''
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    //Map* mpMap;
    // '''
    // mpAtlas 是 ORB-SLAM3 系统中一个非常重要的成员变量,它指向一个 Atlas 类型的指针。
    // Atlas 是 ORB-SLAM3 中用于管理和存储地图数据的核心类。它的主要功能包括:
    // 地图管理:
    // Atlas 负责维护整个SLAM系统的地图数据,包括地图中的特征点、地标、关键帧等信息。
    // 它提供了添加、删除、更新地图元素的接口,确保地图数据的一致性和完整性。
    // 多地图管理:
    // ORB-SLAM3 支持同时管理多个局部地图(子地图),以应对大规模环境建图的需求。
    // Atlas 负责协调这些子地图的融合和切换,为上层应用提供一个统一的地图视图。
    // 地图持久化:
    // Atlas 提供了保存和加载地图数据的接口,支持地图的持久化存储和跨会话的重用。
    // 这样可以避免每次启动SLAM系统时都需要从头重建地图,提高系统的效率和可靠性。
    // 回环检测和优化:
    // Atlas 会跟踪当前的相机位姿,并与地图数据进行对比,检测是否发生了回环。
    // 一旦检测到回环,Atlas 会触发全局优化过程,以修正累积的位姿误差。
    // '''
    Atlas* mpAtlas;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    Viewer* mpViewer;

    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptViewer;

    // Reset flag
    // 这是一个 std::mutex 类型的成员变量,用于保护 mbReset 和 mbResetActiveMap 这两个标志位的访问。在多线程环境下,对这些标志位的读写需要通过这个互斥量来进行同步,避免数据竞争的发生
    std::mutex mMutexReset;
    // 这是一个布尔类型的标志位,表示整个 ORB-SLAM3 系统是否需要进行重置。当该标志位为 true 时,系统会执行全局重置操作,清空地图、关键帧、特征点等所有的数据。
    bool mbReset;
    // 这也是一个布尔类型的标志位,表示当前活跃的地图是否需要进行重置。与 mbReset 不同的是,mbResetActiveMap 只会重置当前正在使用的地图,而不会影响其他已保存的地图
    bool mbResetActiveMap;

    // Change mode flags
    // 这是一个 std::mutex 类型的成员变量,用于保护 mbActivateLocalizationMode 和 mbDeactivateLocalizationMode 两个标志位的访问。
    // 在多线程环境下,对这些标志位的读写需要通过这个互斥量来进行同步,避免数据竞争的发生。
    std::mutex mMutexMode;
    // 这是一个布尔类型的标志位,表示系统是否需要切换到定位模式。
    bool mbActivateLocalizationMode;
    // 这也是一个布尔类型的标志位,表示系统是否需要退出定位模式,切换回SLAM模式。
    bool mbDeactivateLocalizationMode;

    // Shutdown flag
    // mbShutDown 是一个布尔类型的标志位,当它被设置为 true 时,表示整个 ORB-SLAM3 系统需要关闭
    bool mbShutDown;

    // Tracking state
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;

    //
    string mStrLoadAtlasFromFile;
    string mStrSaveAtlasToFile;

    string mStrVocabularyFilePath;

    Settings* settings_;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
