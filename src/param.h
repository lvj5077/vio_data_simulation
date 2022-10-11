//
// Created by hyj on 17-6-22.
//

#ifndef IMUSIM_PARAM_H
#define IMUSIM_PARAM_H

#include <eigen3/Eigen/Core>

class Param{

public:

    Param();

    // time
    int imu_frequency = 200;
    int cam_frequency = 30;
    double imu_timestep = 1./imu_frequency;
    double cam_timestep = 1./cam_frequency;
    double t_start = 0;
    // double t_end = 180;  // seconds

    double t_end = 30;  // seconds

    // // noise
    // double gyro_bias_sigma = 0.00005;
    // double acc_bias_sigma = 0.0005;

    double gyro_bias_sigma = 8.5e-7;
    double acc_bias_sigma = 1.5e-5;

    double gyro_noise_sigma = 1.5e-4;    // rad/s * 1/sqrt(hz)
    double acc_noise_sigma = 5.3e-4;      //　m/(s^2) * 1/sqrt(hz)

    // double gyro_noise_sigma = 0.015;    // rad/s * 1/sqrt(hz)
    // double acc_noise_sigma = 0.019;      //　m/(s^2) * 1/sqrt(hz)


    // double gyro_bias_sigma = 0.0;
    // double acc_bias_sigma = 0.0;
    // double gyro_noise_sigma = 0.0;
    // double acc_noise_sigma = 0.0;


    double pixel_noise = 0.0;              // 1 pixel noise

    // cam f
    double fx = 484.85;
    double fy = 484.85;
    double cx = 320.10;
    double cy = 223.12;
    double image_w = 640;
    double image_h = 480;

    int feature_num = 150;

    // 外参数
    Eigen::Matrix3d R_bc;   // cam to body
    Eigen::Vector3d t_bc;     // cam to body

    int MIN_DIST = 30;      // 30 pixel

};


#endif //IMUSIM_PARAM_H
