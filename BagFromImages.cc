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
int count_messages_between_time_frames(uint64_t start_time_stamp,uint64_t end_time_stamp,rosbag::Bag& bag_to_merge);

int main(int argc, char **argv){
    ros::init(argc, argv, "BagFromImages");

    if(argc!=9)
    {
		cout<<"Number of inputs="<<argc<<endl;
        cerr << "Usage: rosrun BagFromImages BagFromImages <root_path> <image extension .ext>  "
                "<check_flag ( 1 for data validation and 0 for merging bag) > <bag to be merged> <ouput bag name.bag> <num of imu messages> <num of camera messages> <camera_hz>" << endl;
        return 0;
    }

    ros::start();
	
	std::string directory_path = argv[1];
	std::string cam_directory_path = directory_path.append("cam0/");
	std::string bag2mergefullfile = std::string(argv[1]) + std::string(argv[4]);
	std::string file_extension = argv[2];
	std::string outputBagfullfile = std::string(argv[1])+std::string(argv[5]);
	/*
	int num_imu_msgs = std::stoi(std::string(argv[5]),nullptr,10);
	int num_cam_msgs = std::stoi(std::string(argv[6]),nullptr,10);
	int camera_hz = std::stoi(std::string(argv[7]),nullptr,10);
	*/
	int num_imu_msgs = 400;
	int num_cam_msgs = 160;
	int camera_hz = 160;
	int count = 0;
	
	float ratio = (float_t)num_cam_msgs/(float_t)num_imu_msgs;
	cout<<"Image folder name: "<< directory_path<<endl; 
	cout<<"bag to be merged full file: "<< bag2mergefullfile<<endl; 
	cout<<"output bag full file: "<< outputBagfullfile<<endl; 
	int key;
	cin >> key;
	
    // Vector of paths to image
    vector<string> filenames =
            DUtils::FileFunctions::Dir(cam_directory_path.c_str(), file_extension.c_str(), true);
            
    // bag to merge
	rosbag::Bag bag_to_merge;
	bag_to_merge.open(bag2mergefullfile,rosbag::bagmode::Read);
    cout << "Images: " << filenames.size() << endl;
    cout << "Image name: "<<filenames[0]<<endl;
    cout << "\n Press a key and then enter to proceed "<<endl;
    
    cin >> key;
	uint64_t firstTimeStampOfImage = extract_time_from_filename(filenames[0],4);
	uint64_t lastTimeStampOfImage = extract_time_from_filename(filenames[filenames.size()-1],4);
	// Validate and generate report.
	if (std::stoi(std::string(argv[3]),nullptr,10)==1){
		cout<<"Validating data"<<endl;
		
		uint32_t i = 0;
		
		uint64_t end_time_stamp,start_time_stamp;
		// check for missing IMU messages
		while(i<filenames.size()-1){
			start_time_stamp = extract_time_from_filename(filenames[i],4);
			end_time_stamp = extract_time_from_filename(filenames[i+num_cam_msgs],4);
			int measured_imu_msgs = count_messages_between_time_frames(start_time_stamp,end_time_stamp,bag_to_merge);
			if ( measured_imu_msgs!= num_imu_msgs ){
				cout<<"The "<<i<<"th segment has "<<measured_imu_msgs<<endl;
			}
			i = i+num_cam_msgs;
			cout<<"The "<<i<<"th segment is processed"<<endl;
		}
		
		//diff time stamps	
		float cam_delta_t;
		uint64_t  prev_time_stamp = 0;
		uint64_t  curr_time_stamp = 0;
		cout<<"Processing time between images"<<endl;
		for(size_t i=0;i<filenames.size()-1;i++){
			if(!ros::ok())
				break;
			// Get time stamp from filename
			prev_time_stamp = extract_time_from_filename(filenames[i],4);
			curr_time_stamp =  extract_time_from_filename(filenames[i+1],4);
			cam_delta_t = 1.0/((float_t)camera_hz);
			if (curr_time_stamp - prev_time_stamp > 1.1* cam_delta_t || (curr_time_stamp - prev_time_stamp < .9* cam_delta_t))
				cout << " The time between: " <<i<<" and "<<i+1<<"frames is = "<<curr_time_stamp - prev_time_stamp<<endl;
		}
		
		cout<<"Processing time between imu messages"<<endl;
		for(rosbag::MessageInstance const m: rosbag::View(bag_to_merge)){	
			sensor_msgs::Imu::ConstPtr i = m.instantiate<sensor_msgs::Imu>();
			if (i != nullptr){
				if(count == 0){
					uint64_t curr_time_stamp = i->header.stamp.toSec()*1e6;
					uint64_t prev_time_stamp = i->header.stamp.toSec()*1e6;
				}
				else{
					uint64_t curr_time_stamp = i->header.stamp.toSec()*1e6;
					if (curr_time_stamp - prev_time_stamp > 1.1* cam_delta_t*ratio || (curr_time_stamp - prev_time_stamp < .9* cam_delta_t*ratio))
						cout << " The time between: "<<count<<" and "<<count+1<<"frames is = "<<curr_time_stamp - prev_time_stamp<<endl;
				}
				count = count+1;
			}
		}
	
	}
    // Process Output bag
	else if (std::stoi(std::string(argv[3]),nullptr,10) == 0) {
		rosbag::Bag bag_out(argv[3],rosbag::bagmode::Write);
		cout<<"initialisation done"<<endl;
		for(size_t i=0;i<filenames.size();i++)
		{
			if(!ros::ok())
				break;
			// Get time stamp from filename
			uint64_t timeStampOfImage = extract_time_from_filename(filenames[i],4);
			ros::Time rosTimeStamp = convert_to_ros_time(timeStampOfImage,timeAccuracy::MICROSECONDS);

			// store the sensor message in cvImage
			cv::Mat im = cv::imread(filenames[i],cv::IMREAD_GRAYSCALE);
			cv_bridge::CvImage cvImage;
			cvImage.image = im;
			cvImage.encoding = sensor_msgs::image_encodings::MONO8;
			cvImage.header.stamp = rosTimeStamp;
			bag_out.write("/camera/image_raw",rosTimeStamp,cvImage.toImageMsg());
			cout << i << " / " << filenames.size() << endl;
		}
		
		// Merge the other bag
		count = 0;
		for(rosbag::MessageInstance const m: rosbag::View(bag_to_merge))
		{
			sensor_msgs::Imu::ConstPtr i = m.instantiate<sensor_msgs::Imu>();
			if (i != nullptr)
			{
				uint64_t timeStamp = i->header.stamp.toSec()*1e6;
				if (timeStamp >= firstTimeStampOfImage  && timeStamp <= lastTimeStampOfImage)
				{
					bag_out.write("/imu/imu",i->header.stamp,i);
					cout << " Topics added to bag: " << count++ << endl;
				}
			}
		}
		bag_out.close();
	}
	ros::shutdown();
	return 0;
}
int count_messages_between_time_frames(uint64_t start_timestamp,uint64_t end_timestamp,rosbag::Bag& bag_to_merge){
	int count = 0;
	for(rosbag::MessageInstance const m: rosbag::View(bag_to_merge)){
		sensor_msgs::Imu::ConstPtr i = m.instantiate<sensor_msgs::Imu>();
			if (i != nullptr)
			{
				uint64_t timeStamp = i->header.stamp.toSec()*1e6;
				if (timeStamp <= start_timestamp)
					continue;
				else if (timeStamp > start_timestamp  && timeStamp <= end_timestamp)
				{
					count = count+1;
				}
				else 
					break;
			}
		}
	return count;
}


ros::Time convert_to_ros_time(uint64_t timeStamp,timeAccuracy t){
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

std::vector<std::string> split(const std::string& s, char delimiter){
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}
