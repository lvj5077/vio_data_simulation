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

// MotionData IMU::MotionModel(double t)
// {

//     MotionData data;
//     // param
//     float ellipse_x = 15;
//     float ellipse_y = 20;
//     float z = 1;           // z轴做sin运动
//     float K1 = 10;          // z轴的正弦频率是x，y的k1倍
//     float K = M_PI/ 10;    // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周

//     // translation
//     // twb:  body frame in world frame
//     Eigen::Vector3d position( ellipse_x * cos( K * t) + 5, ellipse_y * sin( K * t) + 5,  z * sin( K1 * K * t ) + 5);
//     Eigen::Vector3d dp(- K * ellipse_x * sin(K*t),  K * ellipse_y * cos(K*t), z*K1*K * cos(K1 * K * t));              // position导数　in world frame
//     double K2 = K*K;
//     Eigen::Vector3d ddp( -K2 * ellipse_x * cos(K*t),  -K2 * ellipse_y * sin(K*t), -z*K1*K1*K2 * sin(K1 * K * t));     // position二阶导数

//     // Rotation
//     double k_roll = 0.1;
//     double k_pitch = 0.2;
//     Eigen::Vector3d eulerAngles(k_roll * cos(t) , k_pitch * sin(t) , K*t );   // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
//     Eigen::Vector3d eulerAnglesRates(-k_roll * sin(t) , k_pitch * cos(t) , K);      // euler angles 的导数

// //    Eigen::Vector3d eulerAngles(0.0,0.0, K*t );   // roll ~ 0, pitch ~ 0, yaw ~ [0,2pi]
// //    Eigen::Vector3d eulerAnglesRates(0.,0. , K);      // euler angles 的导数

//     Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles);         // body frame to world frame
//     Eigen::Vector3d imu_gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;   //  euler rates trans to body gyro

//     Eigen::Vector3d gn (0,0,-9.81);                                   //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
//     Eigen::Vector3d imu_acc = Rwb.transpose() * ( ddp -  gn );  //  Rbw * Rwn * gn = gs

//     data.imu_gyro = imu_gyro;
//     data.imu_acc = imu_acc;
//     data.Rwb = Rwb;
//     data.twb = position;
//     data.imu_velocity = dp;
//     data.timestamp = t;
//     return data;

// }


MotionData IMU::MotionModel(double t)
{

    MotionData data;
    data.timestamp = t;
    
    // param
    float ellipse_x = 7;
    float ellipse_y = 4;
    float z = 0.2;           // z轴做sin运动
    float K1 = 10;          // z轴的正弦频率是x，y的k1倍
    
    float K = M_PI/ 10;    // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周

    double K2 = K*K;

    Eigen::Vector3d position;
    Eigen::Vector3d dp;
    Eigen::Vector3d ddp;

    double k_roll = 20*M_PI/180;
    double k_pitch = 20*M_PI/180;
    double k_yaw = 20*M_PI/180;

    Eigen::Vector3d eulerAngles;
    Eigen::Vector3d eulerAnglesRates;


    // translation
    // twb:  body frame in world frame
    
    double ellipse_init = 1;
    t = t - 5.; // add anothor circular motion for initialisation
    if(t<5){ 
        // initialization
        
        K = 4*M_PI/ 10; 
        
        position.x() = ellipse_init * cos( K * t) - ellipse_x - ellipse_init;
        position.y() = ellipse_init * sin( K * t) + ellipse_y/5. - 0.5*K * ellipse_init ;
        position.z() = ellipse_init * sin( K * t) + z - 0.5*K * ellipse_init ;

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

    if(t>=5 && t <6){ 
        K = 4*M_PI/ 10; 
        // slow down

        position.x() = - ellipse_x;
        position.y() = K * ellipse_init*(t-5.) - 0.5*K * ellipse_init*(t-5.)*(t-5.) + ellipse_y/5. - 0.5*K * ellipse_init ;
        position.z() = K * ellipse_init*(t-5.) - 0.5*K * ellipse_init*(t-5.)*(t-5.) + z - 0.5*K * ellipse_init ;

        dp.x() = 0.0;
        dp.y() = K * ellipse_init*(6.-t);
        dp.z() = K * ellipse_init*(6.-t);

        ddp.x() = 0.0;
        ddp.y() = -K * ellipse_init;
        ddp.z() = -K * ellipse_init;



        eulerAngles.x() = k_roll;
        eulerAngles.y() = k_pitch;
        eulerAngles.z() = 0.0;

        eulerAnglesRates.x() = 0.0;
        eulerAnglesRates.y() = 0.0;
        eulerAnglesRates.z() = 0.0;
    }

    K = M_PI/ 10;
    if(t>=6. && t <9.){ 
        // pause for sycn
        K = 4*M_PI/ 10;
        position.x() = - ellipse_x;
        position.y() = ellipse_y/5.;
        position.z() = z;

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

    
    // circle
    // if(t>=9 && t <10)
    // {
    //     double tempK = M_PI/2.;
    //     position.x() = - ellipse_x;
    //     position.y() = ellipse_y *cos( tempK* (t-9.))/5. ;
    //     position.z() = z ;

    //     dp.x() = 0.0;
    //     dp.y() = -ellipse_y *tempK*sin(tempK * (t-9.))/5. ;
    //     dp.z() = 0.0;

    //     ddp.x() = 0.0;
    //     ddp.y() = -ellipse_y *tempK*tempK*cos(tempK * (t-9.))/5. ;
    //     ddp.z() = 0.0;

    //     eulerAngles.x() = k_roll;
    //     eulerAngles.y() = k_pitch;
        
    //     eulerAngles.z() = 0.5*K*(t-9.0)*(t-9.0);

    //     eulerAnglesRates.x() = 0.0;
    //     eulerAnglesRates.y() = 0.0;
    //     eulerAnglesRates.z() = K*(t-9.0);
    // }

    // if(t>=10)
    // {
    //     position.x() = ellipse_x * cos( K * t) ;
    //     position.y() = ellipse_y * sin( K * t) ;
    //     position.z() = z * cos( K1 * K * t ) ;

    //     dp.x() = - K * ellipse_x * sin(K*t); // 0
    //     dp.y() = K * ellipse_y * cos(K*t);   // 1
    //     dp.z() = -z*K1*K * sin(K1 * K * t);   // 1

    //     ddp.x() = -K2 * ellipse_x * cos(K*t);
    //     ddp.y() = -K2 * ellipse_y * sin(K*t);
    //     ddp.z() = -z*K1*K1*K2 * cos(K1 * K * t);

    //     eulerAngles.x() = k_roll * cos(t-10.);
    //     eulerAngles.y() = k_pitch * cos(t-10.);
    //     eulerAngles.z() = K*(t-10)+0.5*K;

    //     eulerAnglesRates.x() = -k_roll * sin(t-10.);
    //     eulerAnglesRates.y() = -k_pitch * sin(t-10.);
    //     eulerAnglesRates.z() = K;

    // }

    // // sin wave
    // if(t>=9 && t <10)
    // {
    //     double tempK = M_PI/2.;
    //     position.x() = - ellipse_x;
    //     position.y() = ellipse_y/5. ;
    //     position.z() = z ;

    //     dp.x() = 0.0;
    //     dp.y() = 0.0;
    //     dp.z() = 0.0;

    //     ddp.x() = 0.0;
    //     ddp.y() = 0.0;
    //     ddp.z() = 0.0;

    //     eulerAngles.x() = k_roll;
    //     eulerAngles.y() = k_pitch;
        
    //     eulerAngles.z() = 0.0;

    //     eulerAnglesRates.x() = 0.0;
    //     eulerAnglesRates.y() = 0.0;
    //     eulerAnglesRates.z() = 0.0;
    // }

    // double y_width = 0.5;

    // if(t>=10)
    // {
    //     t = t-10.;
    //     position.x() = sin(t)-t - ellipse_x;
    //     position.y() = y_width*sin(t-sin(t)) + ellipse_y/5.;
    //     position.z() = z*cos(2*t);

    //     dp.x() = cos(t) -1; // 0
    //     dp.y() = -y_width*( cos(t) -1 )*cos(t - sin(t) );   // 1
    //     dp.z() = -2*z*sin(2*t);   // 1

    //     ddp.x() = -sin(t);
    //     ddp.y() = y_width *sin(t) *cos(t-sin(t))-y_width*sin(t-sin(t))*(cos(t)-1)*(cos(t)-1);
    //     ddp.z() = -4*z*cos(2*t);

    //     eulerAngles.x() = k_roll * cos(1.2*t);
    //     eulerAngles.y() = k_pitch * cos(0.8*t);
    //     eulerAngles.z() = k_yaw*cos(t)-k_yaw;

    //     eulerAnglesRates.x() = -1.2*k_roll * sin(1.2*t);
    //     eulerAnglesRates.y() = -0.8*k_pitch * sin(0.8*t);
    //     eulerAnglesRates.z() = -k_yaw*sin(t);

    //     t = t+10.;

    // }

// spiral
    if(t>=9 && t <10)
    {
        double tempK = M_PI/2.;
        position.x() = - ellipse_x;
        position.y() = ellipse_y *cos( tempK* (t-9.))/5. ;
        position.z() = z + 0.5*(t-9.0)*(t-9.0);

        dp.x() = 0.0;
        dp.y() = -ellipse_y *tempK*sin(tempK * (t-9.))/5. ;
        dp.z() = t - 9.0;

        ddp.x() = 0.0;
        ddp.y() = -ellipse_y *tempK*tempK*cos(tempK * (t-9.))/5. ;
        ddp.z() = 1.0;

        eulerAngles.x() = k_roll;
        eulerAngles.y() = k_pitch;
        
        eulerAngles.z() = 0.5*K*(t-9.0)*(t-9.0);

        eulerAnglesRates.x() = 0.0;
        eulerAnglesRates.y() = 0.0;
        eulerAnglesRates.z() = K*(t-9.0);
    }

    if(t>=10)
    {
        position.x() = ellipse_x * cos( K * t) ;
        position.y() = ellipse_y * sin( K * t) ;
        position.z() = t + cos( t-10. ) - 11. + z+0.5;

        dp.x() = - K * ellipse_x * sin(K*t); // 0
        dp.y() = K * ellipse_y * cos(K*t);   // 1
        dp.z() = 1 - sin( t-10. );   // 1

        ddp.x() = -K2 * ellipse_x * cos(K*t);
        ddp.y() = -K2 * ellipse_y * sin(K*t);
        ddp.z() = -cos(t-10.);

        eulerAngles.x() = k_roll * cos(t-10.);
        eulerAngles.y() = k_pitch * cos(t-10.);
        eulerAngles.z() = K*(t-10)+0.5*K;

        eulerAnglesRates.x() = -k_roll * sin(t-10.);
        eulerAnglesRates.y() = -k_pitch * sin(t-10.);
        eulerAnglesRates.z() = K;

    }

    // for calibration
    // eulerAngles.x() = 0.0;
    // eulerAngles.y() = 0.0;
    // eulerAngles.z() = 0.0;

    // ddp.x() = 0.0;
    // ddp.y() = 0.0;
    // ddp.z() = 0.0;
    // eulerAnglesRates.x() = 0.0;
    // eulerAnglesRates.y() = 0.0;
    // eulerAnglesRates.z() = 0.0;


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
        
    // #define euler 0

    #ifdef euler
        /// imu 动力学模型 欧拉积分
         Eigen::Vector3d acc_w = Qwb * (imupose.imu_acc) + gw;  // aw = Rwb * ( acc_body - acc_bias ) + gw
         Qwb = Qwb * dq;
         Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;
         Vw = Vw + acc_w * dt;
        
    #else
        // std::cerr << "Begin to compute int with mid_val method." << std::endl;
        /// 中值积分 
        int pre_idx = i-1;
        MotionData pre_pos = imudata[pre_idx];
        // w = 1/2 * ((w_k - bk) + (w_k_1 - bk)) 
        auto w_m = 0.5*(imupose.imu_gyro + pre_pos.imu_gyro);
        Eigen::Quaterniond dq_m;
        Eigen::Vector3d dtheta_half_m =  w_m * dt /2.0;
        dq_m.w() = 1;
        dq_m.x() = dtheta_half_m.x();
        dq_m.y() = dtheta_half_m.y();
        dq_m.z() = dtheta_half_m.z();
        dq_m.normalize();

        auto Qwb_n = Qwb * dq_m;
        Eigen::Vector3d acc_w =  0.5 * (Qwb*(pre_pos.imu_acc)+gw + Qwb_n*(imupose.imu_acc)+ gw);
        // aw =  1/2( Rwb*(acc_body_k - acc_bias) + gw + Rwb_k_1*(acc_body_k_1 - acc_bias) + gw ),
        //   assume acc_bias not change overtime
        
        Pwb = Pwb +Vw * dt + 0.5 *  dt * dt * acc_w;
        Vw = Vw + acc_w * dt;
        // update Qwb;
        Qwb = Qwb_n;

        // 中值积分 done.
    #endif 

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
        // save_points<<imupose.timestamp<<" " << Pwb(0) << " " << Pwb(1) << " " << Pwb(2) <<" " <<Qwb.x() << " " << Qwb.y() << " " << Qwb.z() << " " << Qwb.w() << std::endl;

        // printf("imupose.timestamp: %f \n", imupose.timestamp);

        save_points.precision(9);
        save_points <<imupose.timestamp<<" ";
        save_points.precision(5);
        save_points << Pwb(0) << " " << Pwb(1) << " " << Pwb(2) <<" " <<Qwb.x() << " " << Qwb.y() << " " << Qwb.z() << " " << Qwb.w() <<std::endl;
    }

    std::cout<<"test　end"<<std::endl;

}
