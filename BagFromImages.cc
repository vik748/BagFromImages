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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "BagFromImages");

    if(argc!=4)
    {
        cerr << "Usage: rosrun BagFromImages BagFromImages <path to image directory> <image extension .ext>  <path to output bag>" << endl;
        return 0;
    }

    ros::start();

    // Vector of paths to image
    vector<string> filenames =
            DUtils::FileFunctions::Dir(argv[1], argv[2], true);

    cout << "Images: " << filenames.size() << endl;
    cout << "Image name: "<<filenames[0];
    int key;
    cin >> key;
    double time;
    // Frequency
    //double freq = atof(argv[3]);

    // Output bag
    rosbag::Bag bag_out(argv[3],rosbag::bagmode::Write);

    //ros::Time t = ros::Time::now();
    cout<<"initialisation done"<<endl;

    for(size_t i=0;i<filenames.size();i++)
    {
        if(!ros::ok())
            break;
        // Get time stamp from filename
        uint64_t timeStampOfImage = extract_time_from_filename(filenames[i],4);
        ros::Time rosTimeStamp = convert_to_ros_time(timeStampOfImage,timeAccuracy::MICROSECONDS);

        //cout<<"The time stamp is: "<<to_string(rosTimeStamp.sec)<<endl;
        //ROS_INFO_STREAM<<"ROS Time stamp is : "<<rosTimeStamp;

        // store the sensor message in cvImage
        cv::Mat im = cv::imread(filenames[i],cv::IMREAD_GRAYSCALE);
        cv_bridge::CvImage cvImage;
        cvImage.image = im;
        cvImage.encoding = sensor_msgs::image_encodings::MONO8;
        cvImage.header.stamp = rosTimeStamp;
        bag_out.write("/camera/image_raw",rosTimeStamp,cvImage.toImageMsg());
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

uint64_t extract_time_from_filename(std::string filename,int location_in_string)
{
    std::vector<std::string> filename_without_extension = split(filename,'.');
    cout<<"filename without extension:"<<filename_without_extension.at(0)<<" and file extension: "<<filename_without_extension.at(1)<<endl;
    std::vector<std::string> filename_parts = split(filename_without_extension.at(0),'_');
    cout<<"Image sequence from filename:"<<filename_parts.at(3)<< " and its timestamp : "<<filename_parts.at(4)<<endl;
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
