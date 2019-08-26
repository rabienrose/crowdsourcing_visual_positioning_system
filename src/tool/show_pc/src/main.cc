# include <iostream>
# include <string>

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/filters/random_sample.h>
#include <gflags/gflags.h>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;

DEFINE_string(pcl_file_addr, "", "images ");
using namespace std;

int main(int argc, char** argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    PointCloud::Ptr cloud (new PointCloud);
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud> ("background_pc", 1);
    pcl::io::loadPCDFile<PointType> (FLAGS_pcl_file_addr, *cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RandomSample<pcl::PointXYZ> downSizeFilter(true);
    downSizeFilter.setInputCloud (cloud);
    downSizeFilter.setSample (100000);
    downSizeFilter.filter(*outputCloud);

    PointCloud::Ptr msg (new PointCloud);
    msg->header.frame_id = "map";
    
    int count=0;
    for (size_t i = 0; i < outputCloud->size (); i++) {
        //if (outputCloud->points[i].z>0){
            msg->points.push_back (outputCloud->points[i]);
            count++;
        //}
    
    }
    msg->height = 1;
    msg->width = count;
    std::cout<<"point size: "<<count<<std::endl;

    ros::Rate loop_rate(1);


    while (nh.ok())
    {
        //msg->header.stamp = ros::Time::now().toNSec();
        pub.publish (msg);
        ros::spinOnce ();
        loop_rate.sleep ();
    }
}
