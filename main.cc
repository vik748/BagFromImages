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

#include "Thirdparty/DLib/FileFunctions.h"

#include <fstream>


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

    // Frequency
    double freq = atof(argv[3]);

    // Output bag
    rosbag::Bag bag_out(argv[4],rosbag::bagmode::Write);

    ros::Time t = ros::Time::now();

    const float T=1.0f/freq;
    ros::Duration d(T);
    string fname, tname, fold,line;
    int slash;

    for(size_t i=0;i<filenames.size();i++)
    {
        if(!ros::ok())
            break;

        cv::Mat im = cv::imread(filenames[i],CV_LOAD_IMAGE_COLOR);
        slash = filenames[i].find_last_of("/\\");
        fold = filenames[i].substr(0,slash)+"_masks_txt";
        fname = filenames[i].substr(slash+1);
        tname = fold+"/"+fname.substr(0, fname.size()-4)+".txt";
        //cout << tname << endl;
        ifstream corners_file (tname);
        getline (corners_file,line);
        cout << tname << ":" << line << endl;

        cv_bridge::CvImage cvImage;     
        cvImage.image = im;
        cvImage.encoding = sensor_msgs::image_encodings::RGB8;
        cvImage.header.stamp = t;
        cvImage.header.frame_id = line;
        bag_out.write("/camera/image_raw",ros::Time(t),cvImage.toImageMsg());
        t+=d;
        cout << i << " / " << filenames.size() << endl;
    }

    bag_out.close();

    ros::shutdown();

    return 0;
}
