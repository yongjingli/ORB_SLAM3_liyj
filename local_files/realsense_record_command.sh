# catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 # Unable to find either executable 'empy' or Python module 'em'...  try
# realsense-viewer
source /opt/ros/noetic/setup.sh
source ~/catkin_ws/devel/setup.sh
roslaunch realsense2_camera rs_camera.launch 

# rosrun rqt_reconfigure rqt_reconfigure 然后选择左边的stereo_module，然后找到emitter_enabled选项，选择”关闭”即可

# vim ~/catkin_ws/src/realsense-ros/realsense2_camera/launch/rs_camera.launch
#   <arg name="infra_width"         default="640"/>
#   <arg name="infra_height"        default="480"/>
#   <arg name="enable_infra"        default="true"/>
#   <arg name="enable_infra1"       default="true"/>
#   <arg name="enable_infra2"       default="true"/>
#   <arg name="enable_gyro"         default="true"/>
#   <arg name="enable_accel"        default="true"/>
#   <arg name="unite_imu_method"          default="linear_interpolation"/>


# <arg name="enable_sync"               default="true"/>
# <arg name="align_depth"               default="true"/>
# 前者是让不同传感器数据（depth, RGB, IMU）实现时间同步，即具有相同的 timestamp;
# 后者会增加若干 rostopic，其中我们比较关心的是 /camera/aligned_depth_to_color/image_raw，这里的 depth 图像与 RGB 图像是对齐的

# 这里运行可以设置转发的帧率
# 图像默认30(转发设置为20) imu默认200

# rosrun topic_tools throttle messages /camera/infra1/image_rect_raw 20.0 /camera/left/image_raw
# rosrun topic_tools throttle messages /camera/infra2/image_rect_raw 20.0 /camera/right/image_raw

# rosrun topic_tools throttle messages /camera/imu -1 /imu
# 设置为-1不降低帧率

# rosrun topic_tools throttle messages /camera/imu 200.0 /imu
# IMU(加速度计&陀螺仪)：加速度计帧率:63,250；陀螺仪帧率:200,400



rosbag record /camera/color/image_raw /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw /camera/imu
rosbag record /camera/left/image_raw /camera/right/image_raw /camera/color/image_raw /imu
rosbag play V1_02_medium.bag
rostopic list
rqt_image_view /camera/infra1/image_rect_raw


rosbag play V1_02_medium.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw /imu0:=/imu  # euroc数据集

# 自定义数据运行
./stereo_inertial_euroc ../../Vocabulary/ORBvoc.txt /home/pxn-lyj/Egolee/data/realsense_collect/2024-08-02-14-37-15/mav0/RealSense_D435i.yaml /home/pxn-lyj/Egolee/data/realsense_collect/2024-08-02-14-37-15 /home/pxn-lyj/Egolee/data/realsense_collect/2024-08-02-14-37-15/mav0/cam0/data/timestamps.txt realsense_test_results


# 同时启动多个realsense相机
# https://github.com/IntelRealSense/realsense-ros/issues/2880
# ROS1
# rs_multiple_devices.launch

# ROS2
# rs_multi_camera_launch.py
# 同时启动多个相机
roslaunch realsense2_camera rs_multiple_devices.launch  serial_no_camera1:=238222071801 serial_no_camera2:=043322072179

# 查看相机设备号
# rs-enumerate-devices | grep Serial
# Serial Number                 : 	238222071801
# Asic Serial Number            : 	237523063689
# Serial Number                 : 	043322072179
# Asic Serial Number            : 	043323052796

# 加入一下的设置
    #   <arg name="infra_width"         default="640"/>
    #   <arg name="infra_height"        default="480"/>
    #   <arg name="enable_infra"        default="true"/>
    #   <arg name="enable_infra1"       default="true"/>
    #   <arg name="enable_infra2"       default="true"/>
    #   <arg name="enable_gyro"         default="true"/>
    #   <arg name="enable_accel"        default="true"/>
    #   <arg name="unite_imu_method"    default="linear_interpolation"/>
    #   <arg name="enable_depth"        default="false"/>

rosbag record /camera1/color/image_raw /camera1/infra1/image_rect_raw /camera1/infra2/image_rect_raw /camera1/imu /camera2/color/image_raw /camera2/infra1/image_rect_raw /camera2/infra2/image_rect_raw /camera2/imu


# 多个realsense的时间同步
https://github.com/IntelRealSense/realsense-ros/pull/3102

# 主要分为外部触发和master和slave的模式
https://blog.csdn.net/weixin_46190814/article/details/133894524