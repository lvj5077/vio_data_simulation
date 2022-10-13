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

    // Points points;
    // Lines lines;
    // CreatePointsLines(points, lines);

    const std::string home_path = getenv("HOME");
    const std::string bag_path = home_path + "/imu_features.bag";
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Write);

    ros::Time::init();
    double begin =ros::Time::now().toSec();

    begin = 1234567890.123456789;
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
    std::vector<cv::Point2f> old_pts;
    std::vector<int> seenInPrevFrame;
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > allPoints_word;
    allPoints_word.clear();
    for (double t = params.t_start; t < params.t_end;) {
        // std::cout << "t = " << t << " / "<< params.t_end << std::endl;
        
        if(pub_cam)
        {
            std::vector<cv::Point2f> prev_pts;
            std::vector<cv::Point2f> cur_pts;
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


            std::srand ( unsigned ( std::time(0) ) );

            std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > features_cam;  // 对应的２维图像坐标
            int prev_idx = 0;
            for (int i = 0; i < allPoints_word.size(); i++){
                if(seenInPrevFrame[i] < 0)  // 该点在上一帧中没有被观测到
                    continue;

                Eigen::Vector4d pw = allPoints_word[i];         
                pw[3] = 1;                               
                Eigen::Vector4d pc1 = Twc.inverse() * pw;   // T_wc.inverse() * Pw  -- > point in cam frame
                if(pc1(2) < 0 || pc1(2) > 15){ // z必须大于０,在摄像机坐标系前方
                    seenInPrevFrame[i] = -1;
                    prev_idx++;
                    continue;
                }                    

                cv::Point2f pt;
                pt.x = pc1(0)/pc1(2) * params.fx + params.cx;
                pt.y = pc1(1)/pc1(2) * params.fy + params.cy;

                // pt.x = pt.x + dist(generator);
                // pt.y = pt.y + dist(generator);

                if( pt.x < params.image_w && pt.x > 0 && pt.y > 0 && pt.y < params.image_h )
                {
                    cur_pts.push_back(pt);
                    prev_pts.push_back(old_pts[prev_idx]);
                }else{
                    seenInPrevFrame[i] = -1;
                }
                prev_idx++;
            }

            // std::vector<int> oldids;
            // std::vector<cv::Point2f> cur_pts_new;
            // for (int i = 0; i < allPoints_word.size(); i++){
            //     Eigen::Vector4d pw = allPoints_word[i];         
            //     pw[3] = 1;                               
            //     Eigen::Vector4d pc1 = Twc.inverse() * pw;   // T_wc.inverse() * Pw  -- > point in cam frame

            //     if(pc1(2) < 0 || pc1(2) > 15){ // z必须大于０,在摄像机坐标系前方
            //         if(seenInPrevFrame[i] > 0){
            //             prev_idx++;
            //         }
            //         seenInPrevFrame[i] = -1;
            //         continue;
            //     }                    

            //     cv::Point2f pt;
            //     pt.x = pc1(0)/pc1(2) * params.fx + params.cx;
            //     pt.y = pc1(1)/pc1(2) * params.fy + params.cy;

            //     if( pt.x < params.image_w && pt.x > 0 && pt.y > 0 && pt.y < params.image_h ){
            //         if (seenInPrevFrame[i] > 0){
            //             // tracked feature
            //             cur_pts.push_back(pt);
            //             prev_pts.push_back(old_pts[prev_idx]);
            //         }else{
            //             // existing feature but not recognized in this frame
            //             oldids.push_back(i);
            //             cur_pts_new.push_back(pt);
            //         }
            //     }else{
            //         if(seenInPrevFrame[i] > 0){
            //             prev_idx++;
            //         }                    
            //         seenInPrevFrame[i] = -1;
            //     }

            // }

            // // reusing old Points_word, insert to the end
            // // todo: pair<vector<int>, Eigen::Vector4d>  id, point
            // if(oldids.size()>0){
            //     for(int i = 0; i < oldids.size(); i++){
            //         allPoints_word.push_back(  allPoints_word[ oldids[i] ]  );
            //         cur_pts.push_back(cur_pts_new[i]);
            //         seenInPrevFrame.push_back(1);
            //     }
            // }


            // cur_pts size
            // printf("tracked cur_pts size: %ld || need %ld more featurs  \n", cur_pts.size(), params.feature_num - cur_pts.size());

            if(cur_pts.size() < 100 ){
                printf("Warning: too few features tracked || %ld \n", cur_pts.size());
            }

            cv::Mat trackImg = cv::Mat::zeros(params.image_h, params.image_w, CV_8UC3);
            for (size_t i = 0; i < cur_pts.size(); ++i) {
                circle(trackImg, prev_pts[i], 2, cv::Scalar(255,0,0), 2);
                circle(trackImg, cur_pts[i], 2, cv::Scalar(0,0,255), 2);
                cv::arrowedLine(trackImg,  prev_pts[i], cur_pts[i], cv::Scalar(0,255, 0), 1, 8, 0, 0.2);
            }

            cv::Mat mask = cv::Mat(params.image_h, params.image_w, CV_8UC1, cv::Scalar(255));

            for (int i = 0; i < cur_pts.size(); i++){
                if (mask.at<uchar>(cur_pts[i]) == 255)
                {
                    cv::circle(mask, cur_pts[i], params.MIN_DIST, 0, -1);
                }
            }

            while(cur_pts.size() < params.feature_num)
            {
                double pt_normal_x = (double)std::rand() / RAND_MAX * 1. - 0.5;
                double pt_normal_y = (double)std::rand() / RAND_MAX * 1. - 0.5;

                cv::Point2f pt;
                pt.x = params.cx + params.fx * pt_normal_x;
                pt.y = params.cy + params.fy * pt_normal_y;

                // pt.x = pt.x + dist(generator);
                // pt.y = pt.y + dist(generator);

                if( pt.x < params.image_w && pt.x > 0 && pt.y > 0 && pt.y < params.image_h && mask.at<uchar>(pt)==255 )
                {
                    cur_pts.push_back(pt);
                    double dpt = (double)std::rand() / RAND_MAX * 10. + 0.1;

                    // set a measurement limit
                    // if(dpt > 5){
                    //     dpt = -1.0;
                    // }else{
                    //     dpt = dpt + dist(generator);
                    // }

                    Eigen::Vector4d p_c;
                    p_c[0] = pt_normal_x * dpt;
                    p_c[1] = pt_normal_y * dpt;
                    p_c[2] = dpt;
                    p_c[3] = 1.;

                    Eigen::Vector4d p_w = Twc * p_c;
                    // printf("p_w: %f %f %f %f \n", p_w(0), p_w(1), p_w(2), p_w(3));
                    // printf("p_c: %f %f %f %f \n", p_c(0), p_c(1), p_c(2), p_c(3));

                    allPoints_word.push_back(p_w);
                    seenInPrevFrame.push_back(1);
                }
            }

            // printf("cur_pts size: %ld || total %ld featurs \n", cur_pts.size(), allPoints_word.size());

            // std::cout << "==============================" << std::endl;

            int cur_idx = 0;
            for (int i = 0; i < allPoints_word.size(); i++){
                if(seenInPrevFrame[i]<0) continue;

                Eigen::Vector4d pw = allPoints_word[i];         
                pw[3] = 1;                               
                Eigen::Vector4d pc1 = Twc.inverse() * pw;   // T_wc.inverse() * Pw  -- > point in cam frame
                if(pc1(2) < 0){
                    std::cout << "pc1(2) < 0 should not be here!!!" << std::endl;
                }
                
                // if(i > 147){
                //     printf("pw: %f %f %f %f \n", pw(0), pw(1), pw(2), pw(3));
                //     printf("pc1: %f %f %f %f \n", pc1(0), pc1(1), pc1(2), pc1(3));
                // }
                

                double u = cur_pts[cur_idx].x;
                double v = cur_pts[cur_idx].y;

                id_of_point.values.push_back(i);

                u_of_point.values.push_back( u );
                v_of_point.values.push_back( v );

                Eigen::Vector2d obs;
                obs.x() = (u - params.cx) / params.fx;
                obs.y() = (v - params.cy) / params.fy;
                

                // printf("uv: %f, %f ", u, v);
                // printf("obs: %f, %f ", obs.x(), obs.y());
                // printf("pc1(0)/pc1(2), pc1(1)/pc1(2) : %f, %f \n", pc1(0)/pc1(2), pc1(1)/pc1(2));

                features_cam.push_back(obs);
                geometry_msgs::Point32 p;
                p.x = obs.x();
                p.y = obs.y();
                p.z = pc1(2); // depth || used to be 1.0. but used for depth now

                // // set a measurement limit
                // if(p.z > 5){
                //     p.z = -1.0;
                // }else{
                //     // p.z = p.z + dist(generator);
                // } 

                feature_points->points.push_back(p);

                double velocity_x = 0.0;
                double velocity_y = 0.0;
                if(t_cam < 0.0001){
                    velocity_x = 0.0;
                    velocity_y = 0.0;
                }
                if(i < prev_pts.size())
                {
                    velocity_x = (cur_pts[i].x - prev_pts[i].x )*params.cam_frequency / params.fx;
                    velocity_y = (cur_pts[i].y - prev_pts[i].y )*params.cam_frequency / params.fy;
                }

                velocity_x_of_point.values.push_back(velocity_x);
                velocity_y_of_point.values.push_back(velocity_y);



                cur_idx++;
            }


            feature_points->channels.push_back(id_of_point);
            feature_points->channels.push_back(u_of_point);
            feature_points->channels.push_back(v_of_point);
            feature_points->channels.push_back(velocity_x_of_point);
            feature_points->channels.push_back(velocity_y_of_point);

            features_camPre = features_cam;
            

            
            bag.write("/feature_tracker/feature", time_nowCam, feature_points);

            printf("feature_points->header.stamp.toSec():   %f\n", feature_points->header.stamp.toSec());

            if(t_cam < 0.0001){
                prev_pts = cur_pts;
            }
            
            cv_bridge::CvImage track_msg; 
            track_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3; 
            track_msg.header.stamp = time_nowCam;
            track_msg.image = trackImg; 
            // bag.write("/feature_tracker/feature_img", time_nowCam, track_msg); 

            pub_cam = false;
            old_pts = cur_pts;
        }


        // create imu data && add imu noise
        MotionData data = imuGen.MotionModel(t);
        // data.timestamp = t + begin;
        data.timestamp = t;
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
        
        printf("imu_data.header.stamp =                 %f\n", imu_data.header.stamp.toSec());
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
    save_Pose_asTUM("/home/jin/house_model/cam_pose.txt", camdata);
    save_Pose("/home/jin/house_model/imu_noise_All.txt", imudata_noise);
    save_Pose("/home/jin/house_model/imu_All.txt", imudata);

    imuGen.testImu(imudata, "/home/jin/house_model/imu_int_pose.txt");     // test the imu data, integrate the imu data to generate the imu trajecotry
    imuGen.testImu(imudata_noise, "/home/jin/house_model/imu_int_pose_noise.txt");

    std::cout << "Done, save to " << bag_path <<std::endl;
    return 0;
}
