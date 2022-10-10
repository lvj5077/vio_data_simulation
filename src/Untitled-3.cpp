//
// Created by hyj on 18-1-19.
//
#include <random>
#include "imu.h"
#include "utilities.h"

// euler2Rotation:   body frame to interitail frame
Eigen::Matrix3d euler2Rotation( Eigen::Vector3d  eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);
    double yaw = eulerAngles(2);

    double cr = cos(roll); double sr = sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);
    double cy = cos(yaw); double sy = sin(yaw);

    Eigen::Matrix3d RIb;
    RIb<< cy*cp ,   cy*sp*sr - sy*cr,   sy*sr + cy* cr*sp,
            sy*cp,    cy *cr + sy*sr*sp,  sp*sy*cr - cy*sr,
            -sp,         cp*sr,           cp*cr;
    return RIb;
}

Eigen::Matrix3d eulerRates2bodyRates(Eigen::Vector3d eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);

    double cr = cos(roll); double sr = sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);

    Eigen::Matrix3d R;
    R<<  1,   0,    -sp,
            0,   cr,   sr*cp,
            0,   -sr,  cr*cp;

    return R;
}


IMU::IMU(Param p): param_(p)
{
    gyro_bias_ = Eigen::Vector3d::Zero();
    acc_bias_ = Eigen::Vector3d::Zero();
}

void IMU::addIMUnoise(MotionData& data)
{
    std::random_device rd;
    std::default_random_engine generator_(rd());
    std::normal_distribution<double> noise(0.0, 1.0);

    Eigen::Vector3d noise_gyro(noise(generator_),noise(generator_),noise(generator_));
    Eigen::Matrix3d gyro_sqrt_cov = param_.gyro_noise_sigma * Eigen::Matrix3d::Identity();
    data.imu_gyro = data.imu_gyro + gyro_sqrt_cov * noise_gyro / sqrt( param_.imu_timestep ) + gyro_bias_;

    Eigen::Vector3d noise_acc(noise(generator_),noise(generator_),noise(generator_));
    Eigen::Matrix3d acc_sqrt_cov = param_.acc_noise_sigma * Eigen::Matrix3d::Identity();
    data.imu_acc = data.imu_acc + acc_sqrt_cov * noise_acc / sqrt( param_.imu_timestep ) + acc_bias_;

    // gyro_bias update
    Eigen::Vector3d noise_gyro_bias(noise(generator_),noise(generator_),noise(generator_));
    gyro_bias_ += param_.gyro_bias_sigma * sqrt(param_.imu_timestep ) * noise_gyro_bias;
    data.imu_gyro_bias = gyro_bias_;

    // acc_bias update
    Eigen::Vector3d noise_acc_bias(noise(generator_),noise(generator_),noise(generator_));
    acc_bias_ += param_.acc_bias_sigma * sqrt(param_.imu_timestep ) * noise_acc_bias;
    data.imu_acc_bias = acc_bias_;

}

MotionData IMU::MotionModel(double t)
{

    MotionData data;
    // param
    float ellipse_x = 15;
    float ellipse_y = 20;
    float z = 1;           // z轴做sin运动
    float K1 = 10;          // z轴的正弦频率是x，y的k1倍
    float K = M_PI/ 10;    // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周

    double K2 = K*K;

    Eigen::Vector3d position;
    Eigen::Vector3d dp;
    Eigen::Vector3d ddp;

    double k_roll = 0.1;
    double k_pitch = 0.2;

    Eigen::Vector3d eulerAngles;
    Eigen::Vector3d eulerAnglesRates;


    // translation
    // twb:  body frame in world frame
    double k_yaw = 0.2;
    double ellipse_init = 4;
    if(t<5){ 
        // initialization
        
        K = 4*M_PI/ 10; 
        
        position.x() = ellipse_init * cos( K * t) + 5. + ellipse_x - ellipse_init;
        position.y() = ellipse_init * sin( K * t) + 5. ;
        position.z() = ellipse_init * sin( K * t) + 5. ;

        dp.x() = - K * ellipse_init * sin(K*t);
        dp.y() = K * ellipse_init * cos(K*t);
        dp.z() = K * ellipse_init * cos(K*t);

        ddp.x() = -K*K * ellipse_init * cos(K*t);
        ddp.y() = -K*K * ellipse_init * sin(K*t);
        ddp.z() = -K*K * ellipse_init * sin(K*t);

        
        eulerAngles.x() = k_roll * cos(K*t);
        eulerAngles.y() = k_pitch * cos(K*t);
        eulerAngles.z() = k_yaw * cos(K*t) - k_yaw;

        eulerAnglesRates.x() = -k_roll * K * sin(K*t);
        eulerAnglesRates.y() = -k_pitch * K * sin(K*t);
        eulerAnglesRates.z() = -k_yaw * K * sin(K*t);
    }

    if(t>=5 && t <6.25){ 
        K = 4*M_PI/ 10; 
        // slow down

        position.x() = 20.;
        position.y() = ellipse_init * sin( K * t) + 5. ;
        position.z() = ellipse_init * sin( K * t) + 5. ;

        dp.x() = 0.0;
        dp.y() = K * ellipse_init * cos(K*t);
        dp.z() = K * ellipse_init * cos(K*t);

        ddp.x() = 0.0;
        ddp.y() = -K*K * ellipse_init * sin(K*t);
        ddp.z() = -K*K * ellipse_init * sin(K*t);



        eulerAngles.x() = k_roll;
        eulerAngles.y() = k_pitch;
        eulerAngles.z() = 0.0;

        eulerAnglesRates.x() = 0.0;
        eulerAnglesRates.y() = 0.0;
        eulerAnglesRates.z() = 0.0;
    }

    if(t>=6.25 && t <15.){ 
        // pause for sycn
        position.x() = 20.;
        position.y() = 5. + 4.;
        position.z() = 5. + 4.;

        dp.x() = 0.0;
        dp.y() = 0.0;
        dp.z() = 0.0;

        ddp.x() = 0.0;
        ddp.y() = 0.0;
        ddp.z() = 0.0;

        eulerAngles.x() = k_roll;
        eulerAngles.y() = k_pitch;
        eulerAngles.z() = 0.0;

        eulerAnglesRates.x() = 0.0;
        eulerAnglesRates.y() = 0.0;
        eulerAnglesRates.z() = 0.0;
    }

    K = M_PI/ 10;
    double t_temp = t - 15.;

    if(t>=15 && t <20)
    {
        position.x() = ellipse_x * cos( K * t) + 5 + ellipse_x;
        position.y() = ellipse_y * sin( K * t) + 5;
        position.z() = z * sin( K * t ) + 5;

        dp.x() = - K * ellipse_x * sin(K*t);
        dp.y() = K * ellipse_y * cos(K*t);
        dp.z() = z*K1*K * cos( K * t);

        ddp.x() = 0.0;
        ddp.y() =  -K2 * ellipse_y * sin(K*t);
        ddp.z() = -z*K1*K2 * sin( K * t);

        k_roll = 0.1;
        k_pitch = 0.2;


        eulerAngles.x() = k_roll * cos(K*t_temp);
        // eulerAngles.y() = k_pitch * sin(K*t);
        eulerAngles.y() = k_pitch * cos(K*t_temp);
        
        eulerAngles.z() = K*t_temp*t_temp/20.;

        eulerAnglesRates.x() = -k_roll * K * sin(K*t_temp);
        // eulerAnglesRates.y() = k_pitch * K * cos(K*t);
        eulerAnglesRates.y() = -k_pitch * K * sin(K*t_temp);
        eulerAnglesRates.z() = K*t_temp/5.;
    }

    if(t>=20)
    {
        position.x() = ellipse_x * cos( K * t) + 5 + ellipse_x;
        position.y() = ellipse_y * sin( K * t) + 5;
        position.z() = z * sin( K1 * K * t ) + 5;

        dp.x() = - K * ellipse_x * sin(K*t);
        dp.y() = K * ellipse_y * cos(K*t);
        dp.z() = z*K1*K * cos(K1 * K * t);

        ddp.x() = -K2 * ellipse_x * cos(K*t);
        ddp.y() =  -K2 * ellipse_y * sin(K*t);
        ddp.z() = -z*K1*K1*K2 * sin(K1 * K * t);

        k_roll = 0.1;
        k_pitch = 0.2;

        eulerAngles.x() = k_roll * cos(K*t_temp);
        eulerAngles.y() = k_pitch * cos(K*t_temp);

        eulerAngles.z() = K * (t-20.) + 2.5*K;

        eulerAnglesRates.x() = -k_roll * K * sin(K*t_temp);
        eulerAnglesRates.y() = -k_pitch * K * sin(K*t_temp);
        eulerAnglesRates.z() = K;
    }

    // Eigen::Vector3d position( ellipse_x * cos( K * t) + 5, ellipse_y * sin( K * t) + 5,  z * sin( K1 * K * t ) + 5);
    // Eigen::Vector3d dp(- K * ellipse_x * sin(K*t),  K * ellipse_y * cos(K*t), z*K1*K * cos(K1 * K * t));              // position导数　in world frame
    // Eigen::Vector3d ddp( -K2 * ellipse_x * cos(K*t),  -K2 * ellipse_y * sin(K*t), -z*K1*K1*K2 * sin(K1 * K * t));     // position二阶导数

    // Rotation

    // Eigen::Vector3d eulerAngles(k_roll * cos(t) , k_pitch * sin(t) , K*t );   // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
    // Eigen::Vector3d eulerAnglesRates(-k_roll * sin(t) , k_pitch * cos(t) , K);      // euler angles 的导数

//    Eigen::Vector3d eulerAngles(0.0,0.0, K*t );   // roll ~ 0, pitch ~ 0, yaw ~ [0,2pi]
//    Eigen::Vector3d eulerAnglesRates(0.,0. , K);      // euler angles 的导数

    Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles);         // body frame to world frame
    Eigen::Vector3d imu_gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;   //  euler rates trans to body gyro

    Eigen::Vector3d gn (0,0,-9.81);                                   //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
    Eigen::Vector3d imu_acc = Rwb.transpose() * ( ddp -  gn );  //  Rbw * Rwn * gn = gs

    data.imu_gyro = imu_gyro;
    data.imu_acc = imu_acc;
    data.Rwb = Rwb;
    data.twb = position;
    data.imu_velocity = dp;
    data.timestamp = t;
    return data;

}

//读取生成的imu数据并用imu动力学模型对数据进行计算，最后保存imu积分以后的轨迹，
//用来验证数据以及模型的有效性。
void IMU::testImu(std::vector<MotionData>imudata, std::string dist)
{

    std::ofstream save_points;
    save_points.open(dist);

    double dt = param_.imu_timestep;
    Eigen::Vector3d Pwb = init_twb_;              // position :    from  imu measurements
    Eigen::Quaterniond Qwb(init_Rwb_);            // quaterniond:  from imu measurements
    Eigen::Vector3d Vw = init_velocity_;          // velocity  :   from imu measurements
    Eigen::Vector3d gw(0,0,-9.81);    // ENU frame
    Eigen::Vector3d temp_a;
    Eigen::Vector3d theta;
    for (int i = 1; i < imudata.size(); ++i) {

        MotionData imupose = imudata[i];

        //delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
        Eigen::Quaterniond dq;
        Eigen::Vector3d dtheta_half =  imupose.imu_gyro * dt /2.0;
        dq.w() = 1;
        dq.x() = dtheta_half.x();
        dq.y() = dtheta_half.y();
        dq.z() = dtheta_half.z();
        dq.normalize();
        
        /// imu 动力学模型 欧拉积分
        Eigen::Vector3d acc_w = Qwb * (imupose.imu_acc) + gw;  // aw = Rwb * ( acc_body - acc_bias ) + gw
        Qwb = Qwb * dq;
        Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;
        Vw = Vw + acc_w * dt;
        
        /// 中值积分

        //　按着imu postion, imu quaternion , cam postion, cam quaternion 的格式存储，由于没有cam，所以imu存了两次
        // save_points<<imupose.timestamp<<" "
        //            <<Qwb.w()<<" "
        //            <<Qwb.x()<<" "
        //            <<Qwb.y()<<" "
        //            <<Qwb.z()<<" "
        //            <<Pwb(0)<<" "
        //            <<Pwb(1)<<" "
        //            <<Pwb(2)<<" "
        //            <<Qwb.w()<<" "
        //            <<Qwb.x()<<" "
        //            <<Qwb.y()<<" "
        //            <<Qwb.z()<<" "
        //            <<Pwb(0)<<" "
        //            <<Pwb(1)<<" "
        //            <<Pwb(2)<<" "
        //            <<std::endl;
        save_points<<imupose.timestamp<<" " << Pwb(0) << " " << Pwb(1) << " " << Pwb(2) <<" " <<Qwb.x() << " " << Qwb.y() << " " << Qwb.z() << " " << Qwb.w() << std::endl;

    }

    std::cout<<"test　end"<<std::endl;

}
