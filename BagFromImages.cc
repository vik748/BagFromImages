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
#define SECS_TO_NANCOSECS 1000000000

enum timeAccuracy
{   MICROSECONDS = 0,
    NANOSECONDS = 1
};
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "BagFromImages");

    if(argc!=5)
    {
        cerr << "Usage: rosrun BagFromImages BagFromImages <path to image directory> <image extension .ext> <frequency> <path to output bag>" << endl;
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
    rosbag::Bag bag_out(argv[4],rosbag::bagmode::Write);

    ros::Time t = ros::Time::now();

    const float T=1.0f/freq;
    ros::Duration d(T);

    for(size_t i=0;i<filenames.size();i++)
    {
        if(!ros::ok())
            break;
        // Get time stamp from filename
        uint64_t timeStampOfImage = extract_time_from_filename(filenames[i],4);
        ros::Time rosTimeStamp = convert_to_ros_time(timeStampOfImage,timeAccuracy.MICROSECONDS)

        cout<<"The time stamp is: "<<to_string(timeStampOfImage)<<endl;
        ROS_INFO_STREAM<<"ROS Time stamp is : "<<rosTimeStamp;

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
    if(t == timeAccuracy.MICROSECONDS)
    {
        uint32_t seconds = (timeStamp/SEC_TO_MICROSECS);
        uint32_t micro_seconds = (timeStamp%SEC_TO_MICROSECSSEC_TO_MICROSECS);
        ros::Time rosTimeStamp(seconds,microseconds*1000);
    }
    else if (t == timeAccuracy.NANOSECONDS)
    {
        uint32_t seconds = (timeStamp/SEC_TO_NANOSECS);
        uint32_t nano_seconds = (timeStamp%SEC_TO_NANOSECS);
        ros::Time rosTimeStamp(seconds,nano_seconds);
    }
}

uint64_t extract_time_from_filename(std::string filename,int location_in_string)
{
    filename_without_extension = split(filename,'.');
    filename_parts = split(filename_without_extension,'_');
    uint64_t timeStampFromImageName = stol(filename_parts[location_in_string];
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
