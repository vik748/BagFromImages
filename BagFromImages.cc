#include<iostream>
#include<ros/ros.h>
#include<rosbag/bag.h>
#include<rosbag/view.h>
#include<sensor_msgs/Image.h>
#include<std_msgs/Time.h>
#include<std_msgs/Header.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <boost/algorithm/string.hpp>
#include "Thirdparty/DLib/FileFunctions.h"
#define SEC_TO_MICROSECS 1000000
#define SEC_TO_NANOSECS 1000000000

enum timeAccuracy
{   MICROSECONDS = 0,
    NANOSECONDS = 1
};
using namespace std;
ros::Time convert_to_ros_time(uint64_t timeStamp,timeAccuracy t);
std::vector<std::string> split(const std::string& s, char delimiter);
uint64_t extract_time_from_filename(std::string filename,int location_in_string);
void PressEnterToContinue(void);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "BagFromImages");

    if(argc!=8)
    {
        cerr << "Usage: rosrun BagFromImages BagFromImages <root_path> <image extension .ext>  "
                " <bag to be merged> <ouput bag name.bag> <imu hz> <camera_hz>"
                " <trim(0/1)>" <<
                endl;
        return 0;
    }

    ros::start();
    std::string directory_path = argv[1];
    std::string cam_directory_path = directory_path.append("cam0/");
    std::string bag2mergefullfile = std::string(argv[1]) + std::string(argv[3]);
    std::string file_extension = argv[2];
    std::string outputBagfullfile = std::string(argv[1])+std::string(argv[4]);
    int imu_hz = std::stoi(std::string(argv[5]),nullptr,10);
    int camera_hz = std::stoi(std::string(argv[6]),nullptr,10);
    int cut_bag_at_discrepancy = std::stoi(std::string(argv[7]),nullptr,10);
    cout<<"Read all the arguments"<<endl;
    PressEnterToContinue();

    // Display file paths for reference
    cout<<"\nImage folder name: "<< directory_path<<endl;
    cout<<"bag to be merged full file: "<< bag2mergefullfile<<endl;
    cout<<"output bag full file: "<< outputBagfullfile<<endl;
    cout<<"\n Files paths shown"<<endl;
    PressEnterToContinue();

    // compute the parameters that need to be checked
    float ratio = (float_t)camera_hz/(float_t)imu_hz;
    float_t cam_delta_t = 1.0/((float_t)camera_hz)*SEC_TO_MICROSECS; // computed in microseconds
    float_t imu_delta_t = cam_delta_t*ratio; // in microseconds
    cout<<"The camera time period is:" << cam_delta_t <<endl;
    cout<<"The imu time period is:" << imu_delta_t <<endl;
    PressEnterToContinue();

    // Vector of paths to image
    vector<string> filenames = DUtils::FileFunctions::Dir(cam_directory_path\
            .c_str(), file_extension.c_str(), true);

    vector<string> original_filenames = filenames;
    // Bag to merge
    rosbag::Bag bag_to_merge;
    bag_to_merge.open(bag2mergefullfile,rosbag::bagmode::Read);

    // Output bag
    rosbag::Bag bag_out(outputBagfullfile,rosbag::bagmode::Write);
    cout<<"All filepaths for rosbags and images obtained and new bag"
          "path is set"<<endl;
    PressEnterToContinue();

    cout << "Images: " << filenames.size() << endl;
    cout << "Image name: "<<filenames[0]<<endl;

    cout<<"Analyse IMU bag"<<endl;
    PressEnterToContinue();
    double time;
    uint64_t curr_time_stamp=0,prev_time_stamp=0, last_imu_time_stamp;
    uint64_t imu_time_of_discrepancy=0;
    uint64_t cam_time_of_discrepancy=0;
    long valid_time_gap_count = 0;
    long valid_cam_time_gap_count = 0;
    long total_imu_messages = 0; // counted after first camera message
    bool is_first = true;
    // Frequency
    //double freq = atof(argv[3]);
    cout<<"\nCount all IMU messages from first time stamp of camera\n"<<endl;

    uint64_t first_camera_time_stamp = extract_time_from_filename(filenames[0],
            4); // in microseconds
    uint64_t last_camera_time_stamp = extract_time_from_filename
            (filenames[filenames.size()-1],4);

    for(rosbag::MessageInstance const m: rosbag::View(bag_to_merge)) {
        sensor_msgs::Imu::ConstPtr i = m.instantiate<sensor_msgs::Imu>();
        if (i != nullptr) {
            uint64_t time_stamp = i->header.stamp.toSec() * SEC_TO_MICROSECS;
            if (time_stamp >= first_camera_time_stamp) {
                total_imu_messages++;
                last_imu_time_stamp = time_stamp;
                if (is_first) {
                    curr_time_stamp = i->header.stamp.toSec() * SEC_TO_MICROSECS;
                    prev_time_stamp = i->header.stamp.toSec() * SEC_TO_MICROSECS;
                    is_first = false;
                } else {
                    curr_time_stamp = i->header.stamp.toSec() * SEC_TO_MICROSECS;
                    if (curr_time_stamp - prev_time_stamp < 1.1 * imu_delta_t &&
                        curr_time_stamp - prev_time_stamp > .9 * imu_delta_t) {
                        valid_time_gap_count++;
                    }
                    else
                    {
                        imu_time_of_discrepancy = curr_time_stamp;
                    }
                    prev_time_stamp = curr_time_stamp;
                }
            }
        }
    }
    PressEnterToContinue();

    if (imu_time_of_discrepancy)
        cout<<"\nThe IMU messages are coming at equal intervals."
              " All good\n"<<endl;
    else{
        uint64_t time = (imu_time_of_discrepancy-
                    first_camera_time_stamp)/SEC_TO_MICROSECS;
        cout<<"\nSome IMU messages are not coming at equal time intervals"
             ". This happened at "<<time<<"seconds."
                                    "See using rqt_bag"
                                    ""<<endl;
    }

    if (total_imu_messages >= filenames.size()/ratio)
        cout<<"\n Our IMU bag has more IMU messages than the required "
              "messages. All good\n"<<endl;
    if (last_imu_time_stamp > last_camera_time_stamp)
        cout<<"\n IMU was stopped later than the camera. All good\n"<<endl;

    PressEnterToContinue();

    for(int i=0;i<(filenames.size()-1);i++)
    {
        // Time difference between camera messages
        uint64_t curr_img_time_stamp = extract_time_from_filename
                (filenames[i],4);
        uint64_t nxt_img_time_stamp = extract_time_from_filename
                (filenames[i+1],4);

        if (nxt_img_time_stamp - curr_img_time_stamp < 1.1* cam_delta_t||
        (curr_time_stamp - prev_time_stamp > .9* cam_delta_t))
            valid_cam_time_gap_count++;
        else
            cam_time_of_discrepancy = curr_img_time_stamp;
    }

    if (valid_cam_time_gap_count+1 == filenames.size())
        cout<<"\n All camera messages are timestamped at equal intervals. "
              "Good.\n"<<endl;
    else
        cout<<"\n Need some time restamping\n"<<endl;

    // Forming output bag
    for(rosbag::MessageInstance const m: rosbag::View(bag_to_merge)) {
        sensor_msgs::Imu::ConstPtr i = m.instantiate<sensor_msgs::Imu>();
        if (i != nullptr)
        {
            uint64_t timeStamp = i->header.stamp.toSec()*SEC_TO_MICROSECS;
            if (timeStamp >= first_camera_time_stamp  &&
                timeStamp <= last_camera_time_stamp)
                bag_out.write("/imu/imu",i->header.stamp,i);
        }
    }
    cout<<"The imu messages are written to the output bag"<<endl;
    // Find the last camera message
    int end_cam_message;
    if (cut_bag_at_discrepancy)
    {
        last_imu_time_stamp = imu_time_of_discrepancy;
    }


    if (last_imu_time_stamp < last_camera_time_stamp )
        end_cam_message =
                (last_imu_time_stamp-first_camera_time_stamp)/camera_hz;
    else
        end_cam_message = filenames.size();

    cout<<"End cam message ="<<end_cam_message<<endl;
    for(size_t i=0;i<end_cam_message;i++)
    {
        if(!ros::ok())
            break;
        // Writing the time stamps altogether
        uint64_t timeStampOfImage = first_camera_time_stamp +
                                    i*(uint64_t)cam_delta_t;
        ros::Time rosTimeStamp = convert_to_ros_time(timeStampOfImage,
                timeAccuracy::MICROSECONDS);

        // store the sensor message in cvImage
        cv::Mat im = cv::imread(filenames[i],cv::IMREAD_GRAYSCALE);
        cv_bridge::CvImage cvImage;
        cvImage.image = im;
        cvImage.encoding = sensor_msgs::image_encodings::MONO8;
        cvImage.header.stamp = rosTimeStamp;
        bag_out.write("/cam0/image_raw",rosTimeStamp,cvImage.toImageMsg());
        if (i%500==0)
            cout << i << " / " << filenames.size() << endl;
    }
    bag_out.close();
    ros::shutdown();
    return 0;
}

ros::Time convert_to_ros_time(uint64_t timeStamp,timeAccuracy t)
{
    ros::Time rosTimeStamp;
    if(t == timeAccuracy::MICROSECONDS)
    {
        uint32_t seconds = (timeStamp/SEC_TO_MICROSECS);
        uint32_t microseconds = (timeStamp%SEC_TO_MICROSECS);
        rosTimeStamp = ros::Time(seconds,microseconds*1000);
    }
    else if (t == timeAccuracy::NANOSECONDS)
    {
        uint32_t seconds = (timeStamp/SEC_TO_NANOSECS);
        uint32_t nanoseconds = (timeStamp%SEC_TO_NANOSECS);
        rosTimeStamp = ros::Time(seconds,nanoseconds);
    }
    return rosTimeStamp;
}

uint64_t extract_time_from_filename(std::string full_filename,int location_in_string){
    std::vector<std::string> filename_without_extension = split(full_filename,'.');
    //cout<<"filename without extension:"<<filename_without_extension.at(0)<<" and file extension: "<<filename_without_extension.at(1)<<endl;
    std::vector<std::string> full_path = split(filename_without_extension.at(0),'/');
    std::vector<std::string> filename_parts = split(full_path.at(full_path.size()-1),'_');
    //cout<<"Image sequence from filename:"<<filename_parts.at(3)<< " and its timestamp : "<<filename_parts.at(4)<<endl;
    std::string::size_type sz;
    uint64_t timeStampFromImageName = std::stol(filename_parts.at(location_in_string),&sz);

    return timeStampFromImageName;
}

std::vector<std::string> split(const std::string& s, char delimiter)
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}

void PressEnterToContinue(void){
    printf("Press ENTER to continue: ");
    char c=getchar();
    while (c != '\n')
        c=getchar();
}