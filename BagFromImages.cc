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

int debug;
enum timeAccuracy
{   MICROSECONDS = 0,
    NANOSECONDS = 1
};
using namespace std;
ros::Time convert_to_ros_time(uint64_t timeStamp,timeAccuracy t);
std::vector<std::string> split(const std::string& s, char delimiter);
uint64_t extract_time_from_filename(std::string filename,int location_in_string);
int count_messages_between_time_frames(uint64_t start_time_stamp,uint64_t end_time_stamp,rosbag::Bag& bag_to_merge);
void PressEnterToContinue(void);
void change_time_in_filename(vector<string>& filenames, int index,uint64_t timestamp);

int main(int argc, char **argv){
	ros::init(argc, argv, "BagFromImages");

	if(argc!=9)
	{
		cout<<"Number of inputs="<<argc<<endl;
		cerr << "Usage: rosrun BagFromImages BagFromImages <root_path> <image extension .ext>  "
				" <bag to be merged> <ouput bag name.bag> <num of imu messages> <num of camera messages> <camera_hz> < debug mode>" << endl;
		return 0;
	}

	ros::start();

	std::string directory_path = argv[1];
	std::string cam_directory_path = directory_path.append("cam0/");
	std::string bag2mergefullfile = std::string(argv[1]) + std::string(argv[3]);
	std::string file_extension = argv[2];
	std::string outputBagfullfile = std::string(argv[1])+std::string(argv[4]);
	
	int num_imu_msgs = std::stoi(std::string(argv[5]),nullptr,10);
	int num_cam_msgs = std::stoi(std::string(argv[6]),nullptr,10);
	int camera_hz = std::stoi(std::string(argv[7]),nullptr,10);
	/*
	int num_imu_msgs = 5;
	int num_cam_msgs = 2;
	int camera_hz = 160;
	*/
	int count = 0;
	int runlength;

	bool is_bad_data = false;
	debug = std::stoi(std::string(argv[8]));
	float ratio = (float_t)num_cam_msgs/(float_t)num_imu_msgs;
	cout<<"Image folder name: "<< directory_path<<endl; 
	cout<<"bag to be merged full file: "<< bag2mergefullfile<<endl; 
	cout<<"output bag full file: "<< outputBagfullfile<<endl; 
	PressEnterToContinue();

	// Vector of paths to image
	vector<string> filenames = DUtils::FileFunctions::Dir(cam_directory_path.c_str(), file_extension.c_str(), true);
	vector<string> original_filenames = filenames;	
	// bag to merge
	rosbag::Bag bag_to_merge;
	bag_to_merge.open(bag2mergefullfile,rosbag::bagmode::Read);
	cout << "Images: " << filenames.size() << endl;
	cout << "Image name: "<<filenames[0]<<endl;
	cout << "\n Press a key and then enter to proceed "<<endl;


	PressEnterToContinue();
	
	// Validate and generate report.
	cout<<"Validating data"<<endl;
		
	//diff time stamps	
	uint64_t  prev_time_stamp = 0;
	uint64_t  curr_time_stamp = 0;
	cout<<"Processing time between images"<<endl;
	float_t cam_delta_t = 1.0/((float_t)camera_hz)*SEC_TO_MICROSECS; // computed in microseconds
	float_t imu_delta_t = cam_delta_t*ratio; // in microseconds
	cout<<"The camera time period is:" << cam_delta_t <<endl;
	cout<<"The imu time period is:" << imu_delta_t <<endl;
	PressEnterToContinue();
	
	/********************************************************/
	
	cout<<"Processing time between imu messages and number of imu messages between camera messages"<<endl;
	int measured_imu_msgs = 0;
	bool is_first = true;
	int j=0;
	uint64_t start_time_stamp;
	uint64_t end_time_stamp;
	uint64_t imu_count = 0;
	uint64_t first_time_stamp = extract_time_from_filename(filenames[0],4); // in microseconds
	uint64_t second_last_time_stamp = extract_time_from_filename(filenames[filenames.size()-2],4);
	uint64_t last_imu_time_stamp,first_imu_time_stamp;
	// Validating imu data with camera data
	for(rosbag::MessageInstance const m: rosbag::View(bag_to_merge)){
		sensor_msgs::Imu::ConstPtr i = m.instantiate<sensor_msgs::Imu>();
			if (i != nullptr)	{
				uint64_t time_stamp = i->header.stamp.toSec()*SEC_TO_MICROSECS; // ros time is in seconds, converted into microseconds
				// Number of imu messages between two camera messages
				if (time_stamp >= first_time_stamp && time_stamp <= second_last_time_stamp)	{
					imu_count++;
					start_time_stamp = extract_time_from_filename(filenames[j*num_cam_msgs],4); // in microseconds
					end_time_stamp = extract_time_from_filename(filenames[(j+1)*num_cam_msgs],4);
					
					if (debug == 1 || debug == 2 || debug == 3){
						cout<<"j:  "<<j<<endl;
						cout<<"diff time between imu and camera frame:  "<<end_time_stamp-time_stamp<<endl;
						if (debug == 3){
							cout<<"Start time:  "<<start_time_stamp<<" and End Time:  "<<end_time_stamp<<endl;
							cout<<"difference between camera frames  "<<end_time_stamp-start_time_stamp<<endl;
							cout<<"IMU time:  "<<time_stamp<<endl;
							PressEnterToContinue();
						}	
					}
					if (time_stamp >= start_time_stamp  && time_stamp <= end_time_stamp)	{
						//cout<<"IMU time:  "<<time_stamp<<endl;
						measured_imu_msgs++;
						//count = count+1;
							if (time_stamp == end_time_stamp)	{
								if ( (measured_imu_msgs!= num_imu_msgs && j !=0) || (measured_imu_msgs!= num_imu_msgs + 1 && j == 0)  ){
									cout<<"The "<<j<<"th segment has "<<measured_imu_msgs<<" imu messages"<<endl;
									is_bad_data = true;
								}
							j++;
							last_imu_time_stamp = time_stamp;
							measured_imu_msgs = 0;
						}
					}
					//start_time_stamp = end_time_stamp;
				// Time difference between IMU messages	
				if(is_first){
					curr_time_stamp = i->header.stamp.toSec()*SEC_TO_MICROSECS;
					prev_time_stamp = i->header.stamp.toSec()*SEC_TO_MICROSECS;
					first_imu_time_stamp = curr_time_stamp;
					is_first = false;
					}
				else {
					curr_time_stamp = i->header.stamp.toSec()*SEC_TO_MICROSECS;
					if(debug == 1 || debug == 2 || debug == 3){
						cout<<"Debug log: The difference between IMU time stamps is: " <<curr_time_stamp - prev_time_stamp<<endl;
					}
					if (curr_time_stamp - prev_time_stamp > 1.1* imu_delta_t|| curr_time_stamp - prev_time_stamp < .9* imu_delta_t){
						cout << " The time between: "<<count<<" and "<<count+1<<" IMU frames is = "<<curr_time_stamp - prev_time_stamp<<endl;
						is_bad_data = true;
					}
					prev_time_stamp = curr_time_stamp;
					}
				}			
		}
	}
	
	cout<<"Validation completed: Number of IMU messages in camera frames and time stamp difference between imu messages"<<endl;
	cout<<"Total IMU messages"<<imu_count<<endl;
	cout<<" Total segments:"<< j<<endl;
	cout<<"Is data bad? :" << is_bad_data<<endl;
	PressEnterToContinue();
	
	
	
	/*Validate the camera data*/
	uint64_t cam_count = 0;
	if(filenames.size()%2 == 0){ 
		runlength = (int)((filenames.size()-1)/num_cam_msgs); 
	}
	else{ 
		runlength = (filenames.size())/num_cam_msgs;
	}
	for(size_t i=0;i<runlength;i++){
		if(!ros::ok())
			break;
		
		// Get time stamp from filename
		if(num_imu_msgs%num_cam_msgs > 0 ){
			start_time_stamp = extract_time_from_filename(filenames[i*num_cam_msgs],4); 
			end_time_stamp = extract_time_from_filename(filenames[(i+1)*num_cam_msgs],4); 
			for ( int k = 1; k < num_cam_msgs; k++) {
				uint64_t new_time_stamp = start_time_stamp + ((end_time_stamp - start_time_stamp)/num_cam_msgs)*k;
				if (debug == 1 || debug == 2 || debug == 3){
					cout<<"The old time stamps assigned were :"<<extract_time_from_filename(filenames[i*num_cam_msgs + k],4)<<endl;
					cout<<"The new time stamps assigned are :"<<new_time_stamp<<endl;
					cout<<"The difference between new and old time stamp: "<<new_time_stamp - extract_time_from_filename(filenames[i*num_cam_msgs + k],4)<<endl;
					if (debug == 2 || debug == 3)
						PressEnterToContinue();
				}
				change_time_in_filename(filenames,i*num_cam_msgs +k,new_time_stamp);
			
				if (debug ==1 || debug == 2 || debug == 3){
					cout<<"After changing time stamps"<<endl;
					cout<<"The old time stamps assigned were :"<<extract_time_from_filename(filenames[i*num_cam_msgs + k],4)<<endl;
					cout<<"The new time stamps assigned are :"<<new_time_stamp<<endl;
					cout<<"The difference between new and old time stamp: "<<new_time_stamp - extract_time_from_filename(filenames[i*num_cam_msgs + k],4)<<endl;
				}
			}
		}
	}
	for(int i=0;i<runlength*num_cam_msgs;i++){
		cam_count++;
		// Time difference between camera messages
		prev_time_stamp = extract_time_from_filename(filenames[i],4);
		curr_time_stamp = extract_time_from_filename(filenames[i+1],4);
		if (debug == 1 || debug == 2 || debug == 3)
			cout << " The time between: " <<i<<" and "<<i+1<<" camera frames is = "<<curr_time_stamp - prev_time_stamp<<endl;
			
		if (curr_time_stamp - prev_time_stamp > 1.1* cam_delta_t|| (curr_time_stamp - prev_time_stamp < .9* cam_delta_t)) {
			cout << " The time between: " <<i<<" and "<<i+1<<" camera frames is = "<<curr_time_stamp - prev_time_stamp<<endl;
			is_bad_data = true;
		}
	}
	
	cout<<"Validation completed: Time stamp difference between camera frames"<<endl;
	cout<<"Total camera messages"<<cam_count<<endl;
	cout<<"Is data bad? :" << is_bad_data<<endl;	
	PressEnterToContinue();
	
	// Process Output bag
	uint64_t firstTimeStampOfImage = extract_time_from_filename(filenames[0],4);
	uint64_t lastTimeStampOfImage = extract_time_from_filename(filenames[runlength*num_cam_msgs],4);
	cout<<"The first time stamp of camera message :"<<firstTimeStampOfImage<<endl;
	cout<<"The last time stamp of camera message : "<< (lastTimeStampOfImage)<< endl;
	cout<<endl;
	cout<<"The first time stamp of imu message  : "<< first_imu_time_stamp<< endl;
	cout<<"The last time stamp of imu message : "<< (last_imu_time_stamp)<<endl;
	PressEnterToContinue();
	cout<<"The difference between first and last time stamp of camera message : "<< (lastTimeStampOfImage - firstTimeStampOfImage)<< endl;
	cout<<"The difference between first and last time stamp of camera message : "<< (lastTimeStampOfImage - firstTimeStampOfImage)<< endl;

	cout<<"Output bag creation started"<<endl;
	PressEnterToContinue();
	if(!is_bad_data){
		rosbag::Bag bag_out(outputBagfullfile,rosbag::bagmode::Write);
		cout<<"initialisation done"<<endl;
		
		// Add imu bag
		count = 0;
		int count_all =0;
		for(rosbag::MessageInstance const m: rosbag::View(bag_to_merge))
		{
			count_all ++ ;
			sensor_msgs::Imu::ConstPtr i = m.instantiate<sensor_msgs::Imu>();
			if (i != nullptr)
			{
				uint64_t timeStamp = i->header.stamp.toSec()*SEC_TO_MICROSECS;
				
				if (timeStamp >= firstTimeStampOfImage  && timeStamp <= last_imu_time_stamp)
				{
					bag_out.write("/imu/imu",i->header.stamp,i);
					if(debug == 1 ){
						cout << " Topics added to bag: " << count++ << endl;
					}
				}
			}
		}
		
		
		// Add image data
		int total_segments = j;
		for(size_t i=0;i<total_segments*num_cam_msgs;i++)
		{
			if(!ros::ok())
				break;
			// Get time stamp from filename
			uint64_t timeStampOfImage = extract_time_from_filename(filenames[i],4);
			ros::Time rosTimeStamp = convert_to_ros_time(timeStampOfImage,timeAccuracy::MICROSECONDS);

			// store the sensor message in cvImage
			cv::Mat im = cv::imread(original_filenames[i],cv::IMREAD_GRAYSCALE);
			if(! im.data )                              // Check for invalid input
			{
				cout <<  "Error image is empty" << std::endl ;
				break;
			}	
			cv_bridge::CvImage cvImage;
			cvImage.image = im;
			cvImage.encoding = sensor_msgs::image_encodings::MONO8;
			cvImage.header.stamp = rosTimeStamp;
			bag_out.write("/camera_array/cam0/image_raw",rosTimeStamp,cvImage.toImageMsg());
			cout << i << " / " << total_segments*num_cam_msgs << endl;
		}
		
		
		bag_out.close();
		cout<<"counts :"<<count_all<<","<<count;
	}
	ros::shutdown();
	return 0;
}
int count_messages_between_time_frames(uint64_t start_time_stamp,uint64_t end_time_stamp,rosbag::Bag& bag_to_merge){
	int count = 0;
	for(rosbag::MessageInstance const m: rosbag::View(bag_to_merge)){
		sensor_msgs::Imu::ConstPtr i = m.instantiate<sensor_msgs::Imu>();
			if (i != nullptr)
			{
				uint64_t timeStamp = i->header.stamp.toSec()*1e6;
				if (timeStamp <= start_time_stamp)
					continue;
				else if (timeStamp > start_time_stamp  && timeStamp <= end_time_stamp)
				{
					count = count+1;
				}
				else 
					break;
			}
		}
	return count;
}
void change_time_in_filename(vector<string>& filenames, int index,uint64_t timestamp){
	std::vector<std::string> filename_without_extension = split(filenames[index],'.');
    std::vector<std::string> full_path = split(filename_without_extension.at(0),'/');
    std::vector<std::string> filename_parts = split(full_path.at(full_path.size()-1),'_');
    std::string timestr = to_string(timestamp);
    filename_parts.pop_back();
    filename_parts.push_back(timestr); 
    string file = "";
    for ( int i = 0; i < filename_parts.size(); i++){
		if (i< filename_parts.size() -1)
			file = file + filename_parts[i] + "_";
		else
			file = file + filename_parts[i];
		if(debug == 2)
			cout<<"The file name is building as : "<<file<<endl; 
	}
	filenames[index] = file;
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
void PressEnterToContinue(void){ 
	printf("Press ENTER to continue: ");
    char c=getchar();
    while (c != '\n')
     c=getchar(); 
}
