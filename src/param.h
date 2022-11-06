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
    int imu_frequency = 100;
    int cam_frequency = 10;
    double imu_timestep = 1./imu_frequency;
    double cam_timestep = 1./cam_frequency;
    double t_start = 0;
    // double t_end = 180;  // seconds

    // double t_end = 620;  // seconds
    double t_end = 620;  // seconds

    // double gyro_bias_sigma = 0.0;
    // double acc_bias_sigma = 0.0;
    // double gyro_noise_sigma = 0.0;
    // double acc_noise_sigma = 0.0;

// acc_n: 0.01 # 0.119 #0.1  #0.1        # accelerometer measurement noise standard deviation. #0.2   0.04
// gyr_n: 0.003 #0.0208 # 0.01         # gyroscope measurement noise standard deviation.     #0.05  0.004
// acc_w: 0.0015 # 0.000535 #0.001        # accelerometer bias random work noise standard deviation.  #0.02
// gyr_w: 0.000087 #


// # Values from allan plots
// # sequence: dataset-calib-imu-static2.bag (full data range)
// #accelerometer_noise_density: 0.0014     # m/s^1.5
// #accelerometer_random_walk:   0.000086   # m/s^2.5
// #gyroscope_noise_density:     0.000080   # rad/s^0.5
// #gyroscope_random_walk:       0.0000022  # rad/s^1.5

// # Inflated values (to account for unmodelled effects)
// # Those values work well with Kalibr cam-imu calibration.
// #  - white noise multiplied by 2
// #  - bias random walk multiplied by 10
// accelerometer_noise_density: 0.0028     # m/s^1.5
// accelerometer_random_walk:   0.00086    # m/s^2.5
// gyroscope_noise_density:     0.00016    # rad/s^0.5
// gyroscope_random_walk:       0.000022   # rad/s^1.5


// => final results
// accelerometer_noise_density = 0.00051167
// accelerometer_random_walk   = 0.00001698
// gyroscope_noise_density     = 0.00015625
// gyroscope_random_walk       = 0.00000086

    // inflate as TUM VI dataset
    // double acc_noise_sigma = 0.005;      //　m/(s^2) * 1/sqrt(hz)
    // double gyro_noise_sigma = 0.0015;    // rad/s * 1/sqrt(hz)
    // double acc_bias_sigma = 0.00003;
    // double gyro_bias_sigma = 0.000002;

    // Real iphone 12Pro
    double acc_noise_sigma = 0.00051167;      //　m/(s^2) * 1/sqrt(hz)
    double gyro_noise_sigma = 0.00015625;    // rad/s * 1/sqrt(hz)
    double acc_bias_sigma = 0.00001698;
    double gyro_bias_sigma = 0.00000086;


    double pixel_noise = 0.0;              // 1 pixel noise

    // cam f
//     double fx = 460.0;
//     double fy = 460.0;
//     double cx = 320.0;
//     double cy = 240.0;

//    fx: 484.85
//    fy: 484.85
//    cx: 320.10
//    cy: 223.12
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
