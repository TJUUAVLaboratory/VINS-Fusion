
# VINS-fusion使用说明  

## 测试数据集与仿真测试

## 代码结构

- camera_models 多传感器融合时的在校校准
((VINS-fusion)Online Temporal Calibration for Monocular Visual-Inertial Systems)

- loop_fusion

- global_fusion

- vins_estimator


## 修改


    //将body坐标系和camera坐标系旋转一下 ----
    Eigen::AngleAxisd body_camera_rotation (M_PI, Eigen::Vector3d(0,0,1));
    Eigen::Matrix3d  boda_camera_matrax = Eigen::Matrix3d::Identity();
    boda_camera_matrax = body_camera_rotation.matrix() * estimator.ric[0];

    q.setW(Quaterniond(boda_camera_matrax).w());
    q.setX(Quaterniond(boda_camera_matrax).x());
    q.setY(Quaterniond(boda_camera_matrax).y());
    q.setZ(Quaterniond(boda_camera_matrax).z());
    //将body坐标系和camera坐标系旋转一下 ----

这样保证body坐标系与世界坐标系对齐，然后相机坐标系的Z轴指向相机前方


-在xisualization.cpp 中 pubOdometry(), 原本发布的path 时相机坐标系下的，z轴超前，
将其绕x轴旋转-90度，变成z轴朝上，世界坐标系
影响了/vins_node/path话题


## 代码结构

vins-estimator 

add vins_lib  vins_node


订阅输入所需的话题
- img0_callback  img1_callback
- imu_callback
- feature_callback
- restart_callback

实例化一个 Estimator estimator，将所需的数据都送入进去
- estimator.inputImage(time, image0, image1) or  estimator.inputImage(time, image);
- estimator.inputIMU(t, acc, gyr);
- estimator.inputFeature(t, featureFrame);

读取配置参数文件
- estimator.setParameter();



## 多线程

- std::thread sync_thread{sync_process}; 不断往 estimator送入 数据和 图片
- processThread   = std::thread(&Estimator::processMeasurements, this);
    初始化 initFirstIMUPose ==> Rs[0]
    processIMU 预积分 Rs Ps Vs Bas[] Bgs[]
    processImage 各种情况需要考虑



## BA优化
[the goal of bundle adjustment is to find 3D point positions and camera parameters that minimize the reprojection error.]()

> This optimization problem is usually formulated as a non-linear least squares problem, where the error is the squared L2 norm of the difference between the observed feature location and the projection of the corresponding 3D point on the image plane of the camera.

ceres::SizedCostFunction::Evaluate
提供输入的参数， 输出 residuals  jacobians