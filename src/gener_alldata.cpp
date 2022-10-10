#include <ros/ros.h> 
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>


#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp> 

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>

#include <fstream>

#include "imu.h"
#include "utilities.h"

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <random>

using Point = Eigen::Vector4d;
using Points = std::vector<Point, Eigen::aligned_allocator<Point> >;
using Line = std::pair<Eigen::Vector4d, Eigen::Vector4d>;
using Lines = std::vector<Line, Eigen::aligned_allocator<Line> >;

void CreatePointsLines(Points& points, Lines& lines)
{
    std::ifstream f;
    f.open("/home/jin/house_model/house.txt");

    std::srand ( unsigned ( std::time(0) ) );

    while(!f.eof())
    {
        std::string s;
        std::getline(f,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double x,y,z;
            ss >> x;
            ss >> y;
            ss >> z;
            Eigen::Vector4d pt0( x, y, z, 1 );
            ss >> x;
            ss >> y;
            ss >> z;
            Eigen::Vector4d pt1( x, y, z, 1 );

            bool isHistoryPoint = false;
            for (int i = 0; i < points.size(); ++i) {
                Eigen::Vector4d pt = points[i];
                if(pt == pt0)
                {
                    isHistoryPoint = true;
                }
            }
            if(!isHistoryPoint)
                points.push_back(pt0);

            isHistoryPoint = false;
            for (int i = 0; i < points.size(); ++i) {
                Eigen::Vector4d pt = points[i];
                if(pt == pt1)
                {
                    isHistoryPoint = true;
                }
            }
            if(!isHistoryPoint)
                points.push_back(pt1);

            // pt0 = Twl * pt0;
            // pt1 = Twl * pt1;
            lines.emplace_back(pt0, pt1);   // lines
        }
    }

    // create more 3d points, you can comment this code
    int n = points.size();
    for (int j = 0; j < n; ++j) {
        Eigen::Vector4d p = points[j] + Eigen::Vector4d(0.5,0.5,-0.5,0);
        points.push_back(p);
    }

    for (int j = 0; j < 150; ++j) {
        Eigen::Vector4d p;
        p[0] = (double)std::rand() / RAND_MAX * 10;
        p[1] = (double)std::rand() / RAND_MAX * 10;
        p[2] = (double)std::rand() / RAND_MAX * 10;
        p[3] = 1;

        points.push_back(p);
    }

    // save points
    save_points("/home/jin/house_model/all_points.txt", points);
}


int main(int argc, char** argv)
{

    Points points;
    Lines lines;
    CreatePointsLines(points, lines);

    const std::string home_path = getenv("HOME");
    const std::string bag_path = home_path + "/imu_features.bag";
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Write);

    ros::Time::init();
    double begin =ros::Time::now().toSec();

    begin = 1665110771.130929232;
    std::cout << "Start generate data, please waiting..."<<std::endl;

    // IMU model
    Param params;
    IMU imuGen(params);


    std::vector< MotionData > imudata;
    std::vector< MotionData > imudata_noise;
    std::vector< MotionData > camdata;

    double t_cam = params.t_start;
    // std::cout << "t_cam = " << t_cam << std::endl;


    const double mean = 0.0;
    const double stddev = params.pixel_noise;
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);

    bool pub_cam = true;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > features_camPre;
    std::vector<cv::Point2f> prev_pts,cur_pts;
    for (double t = params.t_start; t < params.t_end;) {
        std::cout << "t = " << t << " / "<< params.t_end << std::endl;
        
        if(pub_cam)
        {
            cur_pts.clear();
            ros::Time time_nowCam(begin + t_cam);
            sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
            sensor_msgs::ChannelFloat32 id_of_point;
            sensor_msgs::ChannelFloat32 u_of_point;
            sensor_msgs::ChannelFloat32 v_of_point;
            sensor_msgs::ChannelFloat32 velocity_x_of_point;
            sensor_msgs::ChannelFloat32 velocity_y_of_point;

            feature_points->header.stamp = time_nowCam;
            feature_points->header.frame_id = "world";

            MotionData imu_w = imuGen.MotionModel(t_cam);   // imu body frame to world frame motion
            MotionData cam;

            cam.timestamp = imu_w.timestamp;
            cam.Rwb = imu_w.Rwb * params.R_bc;    // cam frame in world frame
            cam.twb = imu_w.twb + imu_w.Rwb * params.t_bc; //  Tcw = Twb * Tbc ,  t = Rwb * tbc + twb

            camdata.push_back(cam);

            Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
            Twc.block(0, 0, 3, 3) = cam.Rwb;
            Twc.block(0, 3, 3, 1) = cam.twb;

            // 遍历所有的特征点，看哪些特征点在视野里
            std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points_cam;    // ３维点在当前cam视野里
            std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > features_cam;  // 对应的２维图像坐标
            for (int i = 0; i < points.size(); ++i) {
                Eigen::Vector4d pw = points[i];          // 最后一位存着feature id
                pw[3] = 1;                               //改成齐次坐标最后一位
                Eigen::Vector4d pc1 = Twc.inverse() * pw; // T_wc.inverse() * Pw  -- > point in cam frame

                if(pc1(2) < 0) continue; // z必须大于０,在摄像机坐标系前方

                Eigen::Vector2d obs(pc1(0)/pc1(2), pc1(1)/pc1(2)) ;

                // printf("obs: u: %f, v: %f \n", obs(0), obs(1));
                // std::cout << "pc1: " << pc1.transpose() << std::endl;

                double u = obs.x() * params.fx + params.cx;
                double v = obs.y() * params.fy + params.cy;

                // std::cout << "u: " << u << " v: " << v << std::endl;

                u = u + dist(generator);
                v = v + dist(generator);

                // pint u,v
                // std::cout << "u: " << u << " v: " << v << std::endl;

                double un_x, un_y;
                un_x = (u-params.cx)/params.fx;
                un_y = (v-params.cy)/params.fy;

                obs.x() = (u-params.cx)/params.fx;
                obs.y() = (v-params.cy)/params.fy;
                // std::cout << "un_x: " << un_x << " un_y: " << un_y << std::endl;

                // print obs
                // std::cout << "obs: " << obs.transpose() << std::endl;


                double velocity_x = 0.0;
                double velocity_y = 0.0;
                // if( u < params.image_w && u > 0 && v > 0 && v < params.image_h )
                {

                    cv::Point2f temp_uv;
                    temp_uv.x = u;
                    temp_uv.y = v;
                    cur_pts.push_back(temp_uv);

                    points_cam.push_back(points[i]);
                    features_cam.push_back(obs);

                    geometry_msgs::Point32 p;
                    p.x = obs.x();
                    p.y = obs.y();


                    // std::cout << "p: " << p.x << " " << p.y << std::endl << std::endl;
                    // p.x = un_x;
                    // p.y = un_y;
                    
                    // p.z = 1.0;
                    p.z = pc1(2);

                    if(t_cam < 0.0001){
                        velocity_x = 0.0;
                        velocity_y = 0.0;
                    }else{
                        velocity_x = (obs.x() - features_camPre[i].x())*params.cam_frequency;
                        velocity_y = (obs.y() - features_camPre[i].y())*params.cam_frequency;
                    }
                    
                    // print obs u v velocity
                    // printf("p.x: %f, p.y: %f, u: %f, v: %f, velocity_x: %f, velocity_y: %f \n", p.x, p.y, u, v, velocity_x, velocity_y);
                    

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(i);
                    u_of_point.values.push_back(u);
                    v_of_point.values.push_back(v);
                    velocity_x_of_point.values.push_back(velocity_x);
                    velocity_y_of_point.values.push_back(velocity_y);

                    
                }
            }

            features_camPre = features_cam;
            // std::cout << "points => features_cam " << points.size() << " " << features_cam.size() << std::endl;
            // printf("feature_points->header.stamp =  %f \n", feature_points->header.stamp.toSec());
            // std::cout << "feature_points->header.stamp = " << feature_points->header.stamp.toSec() << std::endl;
            
            feature_points->channels.push_back(id_of_point);
            feature_points->channels.push_back(u_of_point);
            feature_points->channels.push_back(v_of_point);
            feature_points->channels.push_back(velocity_x_of_point);
            feature_points->channels.push_back(velocity_y_of_point);
            
            bag.write("/feature_tracker/feature", time_nowCam, feature_points);

            if(t_cam < 0.0001){
                prev_pts = cur_pts;
            }
            
            cv::Mat trackImg = cv::Mat::zeros(params.image_h, params.image_w, CV_8UC3);
            for (size_t i = 0; i < prev_pts.size(); ++i) {
                circle(trackImg, prev_pts[i], 2, cv::Scalar(255,0,0), 2);
                circle(trackImg, cur_pts[i], 2, cv::Scalar(0,0,255), 2);
                cv::arrowedLine(trackImg,  prev_pts[i], cur_pts[i], cv::Scalar(0,255, 0), 1, 8, 0, 0.2);
            }

            cv_bridge::CvImage track_msg; 
            track_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3; 
            track_msg.header.stamp = time_nowCam;
            track_msg.image = trackImg; 
            bag.write("/feature_tracker/feature_img", time_nowCam, track_msg); 

            pub_cam = false;
            prev_pts = cur_pts;
        }


        // create imu data && add imu noise
        MotionData data = imuGen.MotionModel(t);
        data.timestamp = t + begin;
        imudata.push_back(data);
        MotionData data_noise = data;
        imuGen.addIMUnoise(data_noise);
        imudata_noise.push_back(data_noise);

        // to Quaterniond
        Eigen::Quaterniond q(data.Rwb);

        // to ros msg
        ros::Time time_now(begin + t);
        sensor_msgs::Imu imu_data;
        imu_data.header.stamp = time_now;
        imu_data.header.frame_id = "base_link";
        //四元数位姿
        imu_data.orientation.x = q.x();
        imu_data.orientation.y = q.y();
        imu_data.orientation.z = q.z();
        imu_data.orientation.w = q.w();
        // print Quaterniond
        // std::cout << "Quaterniond: " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;


        //线加速度
        imu_data.linear_acceleration.x = data_noise.imu_acc(0); 
        imu_data.linear_acceleration.y = data_noise.imu_acc(1);
        imu_data.linear_acceleration.z = data_noise.imu_acc(2);
        // std::cout << "linear_acceleration: " << data_noise.imu_acc(0) << " " << data_noise.imu_acc(1) << " " << data_noise.imu_acc(2) << std::endl;

        //角速度
        imu_data.angular_velocity.x = data_noise.imu_gyro(0); 
        imu_data.angular_velocity.y = data_noise.imu_gyro(1); 
        imu_data.angular_velocity.z = data_noise.imu_gyro(2);

        // std::cout << "cam.timestamp = " <<  << imu_data.header.stamp.toSec() << std::endl;
        // printf("imu_data.header.stamp =  %f acc.x = %f acc.y = %f acc.z = %f\n", imu_data.header.stamp.toSec(),
        //  imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z);
        bag.write("/imu0", time_now, imu_data);

        t += 1.0/params.imu_frequency;

        if( t > t_cam + 1.0/params.cam_frequency){
            t_cam += 1.0/params.cam_frequency;
            pub_cam = true;
        }

    }
    // fflush(stdout);
    bag.close();
    

    imuGen.init_velocity_ = imudata[0].imu_velocity;
    imuGen.init_twb_ = imudata.at(0).twb;
    imuGen.init_Rwb_ = imudata.at(0).Rwb;
    save_Pose_asTUM("/home/jin/house_model/imu_pose.txt", imudata);
    save_Pose("/home/jin/house_model/imu_pose_All.txt", imudata_noise);
    save_Pose_asTUM("/home/jin/house_model/imu_pose_noise.txt", imudata_noise);

    imuGen.testImu(imudata, "/home/jin/house_model/imu_int_pose.txt");     // test the imu data, integrate the imu data to generate the imu trajecotry
    imuGen.testImu(imudata_noise, "/home/jin/house_model/imu_int_pose_noise.txt");

    std::cout << "Done, save to " << bag_path <<std::endl;
    return 0;
}
