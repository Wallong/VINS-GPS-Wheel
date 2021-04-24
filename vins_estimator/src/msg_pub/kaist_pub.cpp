#include <vector>
#include <thread>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;
using namespace Eigen;

const int nDelayTimes = 2;
string sData_path = "/home/dataset/EuRoC/MH-05/mav0/";
string sConfig_path = "../config/";

bool LoadSensorData(string& sensor_data_file, unordered_map<string, string>* time_data_map)
{
    ifstream data_file(sensor_data_file);
    if (!data_file.is_open())
    {
        cerr << "[LoadSensorData]: Failed to open sensor data file.";
        return false;
    }
    string line_str, time_str;
    while (getline(data_file, line_str))
    {
        stringstream ss(line_str);
        if (!getline(ss, time_str, ','))
        {
            cerr << "[LoadSensorData]: Find a bad line in the encoder file.: " << line_str;
            return false;
        }
        time_data_map->emplace(time_str, line_str);
    }
    return true;
}

int main(int argc, char *argv[])
{
    if(argc != 3)
	{
		cerr << "./run_euroc PATH_TO_FOLDER/MH-05/mav0 PATH_TO_CONFIG/config \n" 
			<< "For example: ./run_euroc /home/stevencui/dataset/EuRoC/MH-05/mav0/ ../config/"<< endl;
		return -1;
	}
    sData_path = argv[1];
    sConfig_path = argv[2];

    ros::init(argc, argv, "KAIST_pub");
    ros::NodeHandle nh("~");
    ros::Publisher imu_publisher;
    ros::Publisher img_publisher;
    imu_publisher = nh.advertise<sensor_msgs::Imu>("/imu/data_raw", 10, true);
    img_publisher = nh.advertise<sensor_msgs::Image>("/stereo/left/image_raw", 5, true);

        unordered_map<string, string> time_encoder_map;
    string encoder_data_path = sData_path + "/sensor_data/encoder.csv";
    if (!LoadSensorData(encoder_data_path, &time_encoder_map))
    {
        cerr << "[PublishData]: Failed to load encoder data.";
        return -1;
    }

    unordered_map<string, string> time_imu_map;
    string imu_data_path = sData_path + "/sensor_data/xsens_imu.csv";
    if (!LoadSensorData(imu_data_path, &time_imu_map))
    {
        cerr << "[PublishData]: Failed to load imu data.";
        return -1;
    }

    string data_stamp_path = sData_path + "/sensor_data/data_stamp.csv";
    ifstream file_data_stamp(data_stamp_path);
    if (!file_data_stamp.is_open())
    {
        cerr << "[PublishData]: Failed to open data_stamp file.";
        return -1;
    }

    vector<string> line_data_vec;
    line_data_vec.reserve(17);
    string line_str, value_str;
    while (getline(file_data_stamp, line_str))
    {
        line_data_vec.clear();
        stringstream ss(line_str);
        while (getline(ss, value_str, ','))
        {
            line_data_vec.push_back(value_str);
        }

        constexpr double kToSecond = 1e-9;
        const string time_str = line_data_vec[0];
        const double timestamp = stod(time_str) * kToSecond;

        const string& sensor_type = line_data_vec[1];
        if (sensor_type == "stereo")
        {
            const string img_file = sData_path + "/image/stereo_left/" + time_str + ".png";
            const cv::Mat raw_image = cv::imread(img_file, CV_LOAD_IMAGE_ANYDEPTH);
            if (raw_image.empty())
            {
                cerr << "[PublishData]: Failed to open image at time: " << time_str;
                return -1;
            }

            cv::Mat color_img;
            cv::cvtColor(raw_image, color_img, cv::COLOR_BayerRG2RGB);

            cv::Mat gray_img;
            cv::cvtColor(color_img, gray_img, cv::COLOR_RGB2GRAY);
            
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray_img).toImageMsg();;
            ros::Time stamp(timestamp);
            img_msg->header.stamp = stamp;

            img_publisher.publish(img_msg);
            // pSystem->PubImageData(timestamp, gray_img);
        }

        if (sensor_type == "imu")
        {
            if (time_imu_map.find(time_str) == time_imu_map.end())
            {
                cerr << "[PublishData]: Failed to find imu data at time: " << time_str;
                return -1;
            }
            const string& imu_str = time_imu_map.at(time_str);
            stringstream imu_ss(imu_str);
            line_data_vec.clear();
            while (getline(imu_ss, value_str, ','))
            {
                line_data_vec.push_back(value_str);
            }

            Vector3d vAcc, vGyr;
            vGyr.x() = std::stod(line_data_vec[8]);
            vGyr.y() = std::stod(line_data_vec[9]);
            vGyr.z() = std::stod(line_data_vec[10]);
            vAcc.x() = std::stod(line_data_vec[11]);
            vAcc.y() = std::stod(line_data_vec[12]);
            vAcc.z() = std::stod(line_data_vec[13]);

            sensor_msgs::Imu imu_msg;
            // pSystem->PubImuData(timestamp, vGyr, vAcc);
            ros::Time stamp(timestamp);
            imu_msg.header.stamp = stamp;
            imu_msg.header.frame_id = "imu_frame";
            imu_msg.orientation.x = std::stod(line_data_vec[1]);
            imu_msg.orientation.y = std::stod(line_data_vec[2]);
            imu_msg.orientation.z = std::stod(line_data_vec[3]);
            imu_msg.orientation.w = std::stod(line_data_vec[4]);
            imu_msg.orientation_covariance[0] = 99999.9;
            imu_msg.orientation_covariance[4] = 99999.9;
            imu_msg.orientation_covariance[8] = 99999.9;
            imu_msg.angular_velocity.x = std::stod(line_data_vec[8]);
            imu_msg.angular_velocity.y = std::stod(line_data_vec[9]);
            imu_msg.angular_velocity.z = std::stod(line_data_vec[10]);
            imu_msg.linear_acceleration.x = std::stod(line_data_vec[11]);
            imu_msg.linear_acceleration.y = std::stod(line_data_vec[12]);
            imu_msg.linear_acceleration.z = std::stod(line_data_vec[13]);
            imu_publisher.publish(imu_msg);
            usleep(5000*nDelayTimes); // usleep 1e-6, 5000*2 = 10000为10ms, 100hz
        }
    }
    ros::spinOnce();
    
    return 0;
}
