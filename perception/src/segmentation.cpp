#include "perception/segmentation.h"

#include "ros/ros.h"
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/common/angles.h"
#include "pcl/common/common.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl_conversions/pcl_conversions.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {
    void SegmentSurface(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices){
        pcl::PointIndices indices_internal;
        pcl::SACSegmentation<PointC> seg;
        seg.setOptimizeCoefficients(true);

        // Search for a plane perpendicular to some axis (specified below).
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);

        // Set the distance to the plane for a point to be an inlier.
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(cloud);

        // Make sure that the plane is perpendicular to Z-axis, 10 degree tolerance.
        Eigen::Vector3f axis;
        axis << 0, 0, 1;
        seg.setAxis(axis);
        seg.setEpsAngle(pcl::deg2rad(10.0));

        // coeff contains the coefficients of the plane:
        // ax + by + cz + d = 0
        pcl::ModelCoefficients coeff;
        seg.segment(indices_internal, coeff);

        
        double distance_above_plane;
        ros::param::param("distance_above_plane", distance_above_plane, 0.005);
        for (size_t i = 0; i < cloud->size(); ++i) {
                const PointC& pt = cloud->points[i];
                float val = coeff.values[0] * pt.x + coeff.values[1] * pt.y +
                coeff.values[2] * pt.z + coeff.values[3];
                if (val <= distance_above_plane) {
                        indices->indices.push_back(i);
                }
        }
        
        // *indices = indices_internal;
        if (indices->indices.size() == 0) {
          ROS_ERROR("Unable to find surface.");
        }
    }
    
void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           pcl::PointIndices::Ptr surface_indices,
                           std::vector<pcl::PointIndices>* object_indices) {
    pcl::ExtractIndices<PointC> extract;
    pcl::PointIndices::Ptr above_surface_indices(new pcl::PointIndices());
    extract.setInputCloud(cloud);
    extract.setIndices(surface_indices);
    extract.setNegative(true);
    extract.filter(above_surface_indices->indices);

    ROS_INFO("There are %ld points above the table", above_surface_indices->indices.size());

    double cluster_tolerance;
    int min_cluster_size, max_cluster_size;
    ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
    ros::param::param("ec_min_cluster_size", min_cluster_size, 10);
    ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);

    pcl::EuclideanClusterExtraction<PointC> euclid;
    euclid.setInputCloud(cloud);
    euclid.setIndices(above_surface_indices);
    euclid.setClusterTolerance(cluster_tolerance);
    euclid.setMinClusterSize(min_cluster_size);
    euclid.setMaxClusterSize(max_cluster_size);
    euclid.extract(*object_indices);

    // Find the size of the smallest and the largest object,
    // where size = number of points in the cluster
    size_t min_size = std::numeric_limits<size_t>::max();
    size_t max_size = std::numeric_limits<size_t>::min();
    for (size_t i = 0; i < object_indices->size(); ++i) {
      size_t cluster_size =object_indices->at(i).indices.size();
      if (cluster_size > max_size) {
        max_size = cluster_size;
      }
      if (cluster_size < min_size) {
        min_size = cluster_size;
      }
    }

    ROS_INFO("Found %ld objects, min size: %ld, max size: %ld",
             object_indices->size(), min_size, max_size);

}

Segmenter::Segmenter(const ros::Publisher& surface_points_pub, const ros::Publisher& marker_pub,
    const ros::Publisher& above_surface) 
    : surface_points_pub_(surface_points_pub), marker_pub_(marker_pub), above_surface_pub_(above_surface){}

void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
  
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);

  pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
  SegmentSurface(cloud, table_inliers);

  for (size_t i=0; i < table_inliers->indices.size(); ++i) {
      int index = table_inliers->indices[i];
      const PointC& pt = cloud->points[index];  
  }

  std::vector<pcl::PointIndices> object_indices;
  SegmentSurfaceObjects(cloud, table_inliers, &object_indices);

  for (size_t i = 0; i < object_indices.size(); ++i) {
      // Reify indices into a point cloud of the object.
      pcl::PointIndices::Ptr indices(new pcl::PointIndices);
      *indices = object_indices[i];
      PointCloudC::Ptr object_cloud(new PointCloudC());
      // TODO: fill in object_cloud using indices
      for (int index = 0 ; index < object_indices.at(i).indices.size(); index++) {
        object_cloud->push_back(cloud->at(object_indices.at(i).indices.at(index)));
      }

      // Publish a bounding box around it.
      visualization_msgs::Marker object_marker;
      object_marker.ns = "objects";
      object_marker.id = i;
      object_marker.header.frame_id = "base_link";
      object_marker.type = visualization_msgs::Marker::CUBE;
      GetAxisAlignedBoundingBox(object_cloud, &object_marker.pose,
                                &object_marker.scale);
      object_marker.color.g = 1;
      object_marker.color.a = 0.3;
      marker_pub_.publish(object_marker);
   }  
        
   // Given these data types:
   PointCloudC::Ptr subset_cloud(new PointCloudC());

   // Extract subset of original_cloud into subset_cloud:
   pcl::ExtractIndices<PointC> extract;
   extract.setInputCloud(cloud);
   extract.setIndices(table_inliers);
   extract.filter(*subset_cloud);
        
   sensor_msgs::PointCloud2 msg_out;
   pcl::toROSMsg(*subset_cloud, msg_out);
   surface_points_pub_.publish(msg_out);

   visualization_msgs::Marker table_marker;
   table_marker.ns = "table";
   table_marker.header.frame_id = "base_link";
   table_marker.type = visualization_msgs::Marker::CUBE;
   GetAxisAlignedBoundingBox(subset_cloud, &table_marker.pose, &table_marker.scale);
   table_marker.color.r = 1;
   table_marker.color.a = 0.8;
   marker_pub_.publish(table_marker);


   // We are reusing the extract object created earlier in the callback.
   PointCloudC::Ptr cloud_out(new PointCloudC());
   sensor_msgs::PointCloud2 msg_out2;
   //extract.setIndices(object_indices);
   extract.setNegative(true);
   extract.filter(*cloud_out);
   pcl::toROSMsg(*cloud_out, msg_out2);
   above_surface_pub_.publish(msg_out2); 

}

void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               geometry_msgs::Pose* pose,
                               geometry_msgs::Vector3* dimensions) {
    PointC min_pcl;
    PointC max_pcl;
    pcl::getMinMax3D<PointC>(*cloud, min_pcl, max_pcl);
    double center_x = (max_pcl.x + min_pcl.x) /2;
    double center_y = (max_pcl.y + min_pcl.y) /2;
    double center_z = (max_pcl.z + min_pcl.z) /2;
    double x_dim = max_pcl.x - min_pcl.x;
    double y_dim = max_pcl.y - min_pcl.y;
    double z_dim = max_pcl.z - min_pcl.z;

    dimensions->x = x_dim;
    dimensions->y = y_dim;
    dimensions->z = z_dim;

    pose->position.x = center_x;
    pose->position.y = center_y;
    pose->position.z = center_z;

    pose->orientation.x = 0;
    pose->orientation.z = 0;
    pose->orientation.y = 0;
    pose->orientation.w = 1;
}


}  // namespace perception
