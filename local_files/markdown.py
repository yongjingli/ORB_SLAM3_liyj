"""
快速跑通案例
https://blog.csdn.net/G_C_H/article/details/136907206
https://zhaoxuhui.top/blog/2020/09/30/orb-slam3-cmake-ros-compilation-and-test.html


代码安装跑通：
# 需要编译安装Pangolin、OpenCV、Eigen3
# opencV的编译安装参考:
可以参考这个的依赖库的安装
 https://zhaoxuhui.top/blog/2017/05/15/Ubuntu%E4%B8%8BSiftGPU%E6%BA%90%E7%A0%81%E7%BC%96%E8%AF%91.html#%E5%9B%9Bubuntu-opencv%E7%8E%AF%E5%A2%83%E6%90%AD%E5%BB%BA

# opencv OpenCV 4.4.0 + OpenCV_Contrib 4.4.0
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.4.0.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.4.0.zip
unzip opencv.zip -d ~/3rdparty
unzip opencv_contrib.zip -d ~/3rdparty

cd ~/3rdparty/opencv-4.4.0 
mkdir build && cd build
# Configure
cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.4.0/modules ..
# Build
cmake --build .
# Install
sudo cmake --install .


https://blog.csdn.net/qq_43318374/article/details/140281336
opencv /usr/bin/ld: /lib/x86_64-linux-gnu/libp11-kit.so.0: undefined referen
cd ~/anaconda3/lib   
# 发现libffi.so.7链接至libffi.so.8.1.0
sudo ln -sf /lib/x86_64-linux-gnu/libffi.so.7.1.0 libffi.so.7
sudo ldconfig


# Pangolin 一个简单高效的 3D 视觉和显示系统,可用于相机标定、三维重建、增强现实等应用
# https://github.com/stevenlovegrove/Pangolin

# 执行./scripts/install_prerequisites.sh recommended 出现下面的问题
E: Unable to locate package catch2

# 从 install_prerequisites.sh 中移除安装 catch2 然后自行安装
git clone https://github.com/catchorg/Catch2.git
cd Catch2
cmake -B build -S . -DBUILD_TESTING=OFF
sudo cmake --build build/ --target install

# Eigen 3.3.0
unzip eigen-3.3.0.zip -d ~/3rdparty
cd ~/3rdparty/eigen-3.3.0
cmake -B build
sudo cmake --install build


# librealsense2
# 如果不安装 librealsense2，运行 ./build.sh 后不会生成 stereo_inertial_realsense_D435i 等与 RealSense 有关的可执行文件

# fatal: unable to access 'https://github.com/.../.git': Could not resolve host: github.com
git config --global --unset http.proxy 
git config --global --unset https.proxy
# 采用realsense-viewer打开realsense


# 在orb3-slam中执行./build.sh出现
error: ‘m_slots’ was not declared in this scope 
/home/pxn-lyj/Egolee/packages/Pangolin/components/pango_core/include/sigslot/
signal.hpp:1180:65: error: ‘slots_reference’ was not declared in this scope

https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/458
sed -i 's/++11/++14/g' CMakeLists.txt

# 运行案例 采用自带realsense
./Examples/Stereo-Inertial/stereo_inertial_realsense_D435i Vocabulary/ORBvoc.txt
 ./Examples/Stereo-Inertial/RealSense_D435i.yaml

 出现segment fault的错误
 [1]    1172248 segmentation fault (core dumped)  ./Examples/Stereo-Inertial/stereo_inertial_realsense_D435i  

 vim src/System.cc 第84行
将cout << (*settings_) << endl;这行注释
mStrLoadAtlasFromFile = settings_->atlasLoadFile();
mStrSaveAtlasToFile = settings_->atlasSaveFile();
//cout << (*settings_) << endl;
 
 出现not enough acceleration，需要移动相机才可以，在相机里是有IMU的
 然后在弹出的界面从选择stop才可以终止程序，不能在终端里采用ctrl+c的方式停止程序

 

采用EuRoc数据进行测试验证
https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
在下载地址中直接点击可能没反应，通过右键的方式获取下载地址，然后通过wget -c的方式进行下载
wget -c http://robotics.ethz.ch/\~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip


# 字典文件、参数设置文件、影像序列文件夹路径、对应的时间戳文件以及最后一个参数
cd /home/pxn-lyj/Egolee/programs/ORB_SLAM3/Examples/Stereo-Inertial
./stereo_inertial_euroc ../../Vocabulary/ORBvoc.txt ./EuRoC.yaml /home/pxn-lyj/Egolee/data/MH_01_easy ./EuRoC_TimeStamps/MH01.txt mh01_results

运行起来会遇到没有可视化界面的情况
vim stereo_inertial_euroc.cc
# 将false修改为true即可可视化成功
// ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO, false);
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO, true);
重新make编译即可展示可视化界面



# 代码里用的是ros1
orb-slam3的ros2版本： https://github.com/zang09/ORB_SLAM3_ROS2
暂时先不考虑ros，先上手再说



# 自定义数据运行
./stereo_inertial_euroc ../../Vocabulary/ORBvoc.txt /home/pxn-lyj/Egolee/data/realsense_collect/2024-08-02-14-37-15/mav0/RealSense_D435i.yaml /home/pxn-lyj/Egolee/data/realsense_collect/2024-08-02-14-37-15 /home/pxn-lyj/Egolee/data/realsense_collect/2024-08-02-14-37-15/mav0/cam0/data/timestamps.txt realsense_test_results
"""


# 理解代码 记录的还不错
# https://blog.csdn.net/qq_39533374/article/details/123709139

# 理解特征点投影和匹配
# https://blog.csdn.net/qq_49561752/article/details/134501848

    # 特别是这段计算两帧的位移关系,对于转换公式的理解有一定的帮助
    # // Step 2 计算当前帧和前一帧的平移向量
    # const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);  //当前帧的相机位姿
    # const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);

    # const cv::Mat twc = -Rcw.t()*tcw;   //当前相机坐标系到世界坐标系的平移向量


    # const cv::Mat Rlw = LastFrame.mTcw.rowRange(0,3).colRange(0,3);     // 上一帧的相机位姿
    # const cv::Mat tlw = LastFrame.mTcw.rowRange(0,3).col(3); // tlw(l)


    # // 当前帧相对于上一帧相机的平移向量
    # const cv::Mat tlc = Rlw*twc+tlw; 
