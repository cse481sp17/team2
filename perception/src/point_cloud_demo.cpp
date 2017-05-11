#include "ros/ros.h"
#include "perception/crop.h"
#include "perception/downsample.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
#include "perception/segmentation.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_demo");
  ros::NodeHandle nh;
 
  ros::Publisher crop_pub =
      nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  ros::Publisher downsampler_pub =
      nh.advertise<sensor_msgs::PointCloud2>("downsampled_cloud", 1, true);

  perception::Cropper cropper(crop_pub);
  perception::DownSampler downsampler(downsampler_pub);

  ros::Subscriber cloud_sub =
      nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);
  ros::Subscriber cropped_sub =
      nh.subscribe("cropped_cloud", 1, &perception::DownSampler::Callback, &downsampler);
  
  ros::Publisher table_pub =
      nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1, true);
  ros::Publisher marker_pub = 
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
  ros::Publisher above_table_pub =
      nh.advertise<sensor_msgs::PointCloud2>("above_cloud", 1, true);

  perception::Segmenter segmenter(table_pub, marker_pub, above_table_pub);
  ros::Subscriber sub =
      nh.subscribe("downsampled_cloud", 1, &perception::Segmenter::Callback, &segmenter);

  ros::spin();
  return 0;
}
