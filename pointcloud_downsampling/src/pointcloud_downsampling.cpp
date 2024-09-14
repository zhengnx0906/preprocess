#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudPreprocessor
{
public:
    PointCloudPreprocessor(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    {
        private_nh.param("input_topic", input_topic_, std::string("/input"));
        private_nh.param("output_topic", output_topic_, std::string("/output"));
        private_nh.param("leaf_size", leaf_size_, 0.1);
        private_nh.param("min_x", min_x_, -1.0);
        private_nh.param("min_y", min_y_, -1.0);
        private_nh.param("min_z", min_z_, -1.0);
        private_nh.param("max_x", max_x_, 1.0);
        private_nh.param("max_y", max_y_, 1.0);
        private_nh.param("max_z", max_z_, 1.0);
        private_nh.param("use_different_frame",use_different_frame , false);
        private_nh.param("frameid",frameid , std::string("origin"));

        
        sub_ = nh.subscribe(input_topic_, 10, &PointCloudPreprocessor::pointCloudCallback, this);
        pub_ = nh.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
    {
        // Convert ROS message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // CropBox filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::CropBox<pcl::PointXYZ> crop;
        crop.setInputCloud(cloud);
        crop.setMin(Eigen::Vector4f(min_x_, min_y_, min_z_, 1.0));
        crop.setMax(Eigen::Vector4f(max_x_, max_y_, max_z_, 1.0));
        crop.filter(*cloud_cropped);

        // VoxelGrid filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_cropped);
        sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        sor.filter(*cloud_filtered);

        // Convert PCL point cloud back to ROS message
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);
        output.header = cloud_msg->header;
        if(use_different_frame) output.header.frame_id=frameid;
        pub_.publish(output);
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
    std::string input_topic_;
    std::string output_topic_;
    double leaf_size_;
    double min_x_, min_y_, min_z_, max_x_, max_y_, max_z_;
    bool use_different_frame;
    std::string frameid; 
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_preprocessor");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    PointCloudPreprocessor preprocessor(nh, private_nh);
    ros::spin();
    return 0;
}
