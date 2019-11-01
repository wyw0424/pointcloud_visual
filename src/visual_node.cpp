// ROS Headers
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <std_msgs/Bool.h>
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include <std_srvs/Trigger.h>

// pcl headers
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/PCLHeader.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <tf/transform_broadcaster.h>
#include <tf/tfMessage.h>
#include <tf2_msgs/TFMessage.h>
#include <visualization_msgs/MarkerArray.h>

// opencv headers
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
typedef std::vector<geometry_msgs::Pose> pose_list;
/*
Adapted from control node trigger method and use new map_precise tf frame on getting pointcloud
*/
class visual_node
{

private:
	ros::NodeHandle node_;

	ros::Subscriber left_laser_sub_;
	ros::Subscriber right_laser_sub_;

	ros::Publisher pointcloud_whole_pub_, pointcloud_right_pub_;
	// Service
	ros::ServiceServer save_data_srv_;

	int save_data_index_;
	bool save_data_status_;
	bool sys_status_;

	tf::TransformListener left_tf_listener_;
	tf::TransformListener right_tf_listener_;

	laser_geometry::LaserProjection projector_;
	std::vector<sensor_msgs::PointCloud2::Ptr> left_buffer_; //buffer to hold scans of left laser scanner
	std::vector<sensor_msgs::PointCloud2::Ptr> right_buffer_; //buffer to hold scans of right laser scanner


	unsigned int left_seq_; // left scan call back sequence number
	unsigned int right_seq_; // right scan call back sequence number
	unsigned int raw_cloud_seq_; // raw point cloud sequence number

	double imu_height_;
	double robot_height_;
	double odom_transform_tolerance_;
	double altitude_threshold_;
	double point_dist_threshold_;
	double point_res_;

	// PCL pointcloudXYZ pointer
	pcl_cloud::Ptr raw_cloud_;
	pcl_cloud::Ptr left_cloud_;
	pcl_cloud::Ptr right_cloud_;

	pcl_cloud::Ptr cloud_filtered;
	pcl_cloud::Ptr sor_cloud_filtered;

	std::string map_frame_;
	std::string dataset_;

	bool writeCloud(const std::string path)
	{
		std::string full_path;
		std::string string_tmp = std::to_string(save_data_index_);

		full_path = path + string_tmp + "_raw_pointcloud.pcd";

		ROS_INFO("writing to cloud file: %s", full_path.c_str());
		pcl::io::savePCDFileBinary(full_path, *raw_cloud_);
		ROS_INFO("Done");

		return true;
	}

public:

	visual_node();
	~visual_node();


	bool saveData(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
	void handleBuffer(const std::vector<sensor_msgs::PointCloud2::Ptr>& buffer, std::string position);
	void leftScanCB(const sensor_msgs::LaserScan::ConstPtr& scan);
	void rightScanCB(const sensor_msgs::LaserScan::ConstPtr& scan);
	// Voxel grid downsample our pointcloud
	void voxelGridDownsample();
	void sorFilter();
	void update();

	//void OdomToMapPreciseCB(const tf::tfMessage::ConstPtr& message);
	//void hectorOdomToBaseTFCB(const tf2_msgs::TFMessage::ConstPtr& message);

};

visual_node::visual_node()
: left_seq_(0)
, right_seq_(0)
, raw_cloud_(new pcl_cloud)
, cloud_filtered(new pcl_cloud)
, sor_cloud_filtered(new pcl_cloud)
, raw_cloud_seq_(0)
, robot_height_(0.0)
, left_cloud_(new pcl_cloud)
, right_cloud_(new pcl_cloud)
, save_data_status_(false)
, save_data_index_(0)
{
	// parameters
	
	std::string left_scan, right_scan;
	ros::param::get("/visual_node/left_scan", left_scan);
	ros::param::get("/visual_node/right_scan", right_scan);
	
	ros::param::get("/visual_node/dataset", dataset_);
	ros::param::get("/visual_node/odom_transform_tolerance", odom_transform_tolerance_);
	ros::param::get("/visual_node/map_frame", map_frame_);


	// subscribers
	//ros::topic::waitForMessage<sensor_msgs::LaserScan>(left_scan);
	left_laser_sub_ = node_.subscribe<sensor_msgs::LaserScan> (left_scan, 100, &visual_node::leftScanCB, this);
	//ros::topic::waitForMessage<sensor_msgs::LaserScan>(right_scan);
	right_laser_sub_ = node_.subscribe<sensor_msgs::LaserScan> (right_scan, 100, &visual_node::rightScanCB, this);

	pointcloud_whole_pub_ = node_.advertise<sensor_msgs::PointCloud2> ("whole_pc2", 1);  
	//pointcloud_right_pub_ = node_.advertise<sensor_msgs::PointCloud2> ("right_pc_map", 1);  


	imu_height_ = 0.0;

	point_dist_threshold_ = 3.0;		// two points distance
	point_res_ = 0.02;
	altitude_threshold_ = 1.00;			// laser scan first point and last point altitude different threshold
										// if bigger than this value: 
										// 1. there must be a hole(glasses) on the floor or 
										// 2. tilted laser or something wrong with our machine(fall down)
										// 3. running machine around edges(second floor can see first floor stuff)

	// Service
	save_data_srv_ = node_.advertiseService("/visual_node/save_data", &visual_node::saveData, this);

}

visual_node::~visual_node()
{
	left_buffer_.clear();
	right_buffer_.clear();
}


bool visual_node::saveData(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
	save_data_status_ = true;
	std::string full_path = dataset_;
	if(writeCloud(full_path))
	{
		save_data_index_++;

		resp.success = true;
		resp.message = "save data succeed";
		return resp.success;
	}
}

void visual_node::handleBuffer(const std::vector<sensor_msgs::PointCloud2::Ptr>& buffer, std::string position)
{
	pcl_cloud::Ptr cloud_pcl (new pcl_cloud() );
	for(unsigned int i = 0; i < buffer.size(); i++)
	{
		pcl::fromROSMsg(*buffer[i], *cloud_pcl);
		// Filling points in blind spot
		// double delta_x, delta_y, delta_z;
		// double steps;
		// double inc_x, inc_y, inc_z;
		// double new_x, new_y, new_z;
		// double two_point_dist;
		// double average_height;

		// // Cartographer use imu as reference frame and can't have translation from imu to base_link.
		// // Therefore, we need to move all laser data in z axis by among of imu offset. (Here M1: 1.62, M2: 1.6995)
	 //    for(size_t j = 0; j < cloud_pcl->points.size(); j++)
	 //    {
	 //      	cloud_pcl->points[j].z = cloud_pcl->points[j].z + imu_height_;
	 //    }


		// delta_z = cloud_pcl->points[0].z - cloud_pcl->points[cloud_pcl->points.size()-1].z;
		// average_height = (cloud_pcl->points[0].z + cloud_pcl->points[cloud_pcl->points.size()-1].z)/2.0;
	
		// if(delta_z < altitude_threshold_)		// two points altitude difference
		// {
		// 	delta_x = cloud_pcl->points[0].x - cloud_pcl->points[cloud_pcl->points.size()-1].x;
		// 	delta_y = cloud_pcl->points[0].y - cloud_pcl->points[cloud_pcl->points.size()-1].y;
		// 	two_point_dist = sqrt(delta_x*delta_x +delta_y*delta_y);
		// 	if(two_point_dist < point_dist_threshold_)		// two points distance
		// 	{
		// 		steps = two_point_dist/point_res_;
		// 		inc_x = delta_x / steps;
		// 		inc_y = delta_y / steps;
		// 		inc_z = delta_z / steps;
		// 		for(unsigned int j = 0; j < steps; j++)
		// 		{
		// 			new_x = cloud_pcl->points[cloud_pcl->points.size()-1].x + inc_x;
		// 			new_y = cloud_pcl->points[cloud_pcl->points.size()-1].y + inc_y;
		// 			new_z = robot_height_;
		// 			cloud_pcl->push_back(pcl::PointXYZ(new_x, new_y, new_z));
		// 		}
		// 	}
		// }
		*raw_cloud_ += *cloud_pcl;

		if(position=="left")
		{
			*left_cloud_ += *cloud_pcl;
		}
		if(position=="right")
		{
			*right_cloud_ += *cloud_pcl;
		}
	}
}

void visual_node::leftScanCB(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	tf::StampedTransform transform;
	//if(
	try
	{
		ros::Time now = ros::Time::now();
	    left_tf_listener_.waitForTransform(scan->header.frame_id, map_frame_, now, ros::Duration(odom_transform_tolerance_));
		left_tf_listener_.lookupTransform(scan->header.frame_id, map_frame_, now, transform);
		sensor_msgs::PointCloud2::Ptr cloud (new sensor_msgs::PointCloud2);
		projector_.transformLaserScanToPointCloud(map_frame_, *scan, *cloud, left_tf_listener_);
		cloud->header.frame_id = map_frame_;
		cloud->header.seq = left_seq_;

		// transfer to PCL
		pcl_cloud::Ptr cloud_pcl (new pcl_cloud() );
		pcl::fromROSMsg(*cloud, *cloud_pcl);
		*raw_cloud_ += *cloud_pcl;

		//std::cout << "getting left cloud.. " << std::endl;
		//left_buffer_.push_back(cloud);
		left_seq_++;
	}
    catch (tf::TransformException &ex) {
      	ROS_WARN("%s",ex.what());
      	ros::Duration(0.1).sleep();
    }


}


void visual_node::rightScanCB(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	tf::StampedTransform transform;
	//if(
	try
	{
		ros::Time now = ros::Time::now();
		right_tf_listener_.waitForTransform(scan->header.frame_id, map_frame_, now, ros::Duration(odom_transform_tolerance_));
		right_tf_listener_.lookupTransform(scan->header.frame_id, map_frame_, now, transform);
		sensor_msgs::PointCloud2::Ptr cloud (new sensor_msgs::PointCloud2);
		projector_.transformLaserScanToPointCloud(map_frame_, *scan, *cloud, right_tf_listener_);
		cloud->header.frame_id = map_frame_;
		cloud->header.seq = right_seq_;

		// transfer to PCL
		pcl_cloud::Ptr cloud_pcl (new pcl_cloud() );
		pcl::fromROSMsg(*cloud, *cloud_pcl);
		*raw_cloud_ += *cloud_pcl;

		//std::cout << "getting right cloud.. " << std::endl;
		//right_buffer_.push_back(cloud);
		right_seq_++;
	}
    catch (tf::TransformException &ex) {
      	ROS_WARN("%s",ex.what());
      	ros::Duration(0.1).sleep();

    }

}


void visual_node::voxelGridDownsample()
{
	ROS_INFO("Filtering pointcloud, please wait....");

	// Voxel grid filtering
	pcl::VoxelGrid<pcl::PointXYZ> vgds;
	vgds.setInputCloud (sor_cloud_filtered);
	vgds.setLeafSize (0.02f, 0.02f, 0.02f);
	// Set pointcloud number per voxel
	vgds.setDownsampleAllData(true);
	
	// pcl 1.8 function
	//vgds.setMinimumPointsNumberPerVoxel(3);
	vgds.filter (*cloud_filtered);
}

void visual_node::sorFilter()
{
	ROS_INFO("Filtering pointcloud, please wait....");

	// StatisticalOutlierRemoval
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (raw_cloud_);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.filter (*sor_cloud_filtered);
}


void visual_node::update()
{
	// handleBuffer(left_buffer_, "left");
	// left_buffer_.clear();
	// handleBuffer(right_buffer_, "right");
	// right_buffer_.clear();
    sensor_msgs::PointCloud2Ptr  whole_ros_pc2(new sensor_msgs::PointCloud2);
    //sensor_msgs::PointCloud2Ptr  right_ros_pc2(new sensor_msgs::PointCloud2);

	pcl::VoxelGrid<pcl::PointXYZ> vgds;
	vgds.setInputCloud (raw_cloud_);
	vgds.setLeafSize (0.02f, 0.02f, 0.01f);
	// Set pointcloud number per voxel
	vgds.setDownsampleAllData(true);
	
	// pcl 1.8 function
	//vgds.setMinimumPointsNumberPerVoxel(3);
	vgds.filter (*cloud_filtered);
	
	pcl::toROSMsg( *cloud_filtered, *whole_ros_pc2 );
	//pcl::toROSMsg( *right_cloud_, *right_ros_pc2 );
	
    whole_ros_pc2->header.frame_id = map_frame_;
    whole_ros_pc2->header.stamp = ros::Time::now();
    //right_ros_pc2->header.frame_id = map_frame_;
    //right_ros_pc2->header.stamp = ros::Time::now();
    pointcloud_whole_pub_.publish(*whole_ros_pc2);
    //pointcloud_right_pub_.publish(*right_ros_pc2);
	raw_cloud_seq_++;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "visual_node");
	visual_node node;

	ROS_INFO("Point cloud generation running!");

	ros::Rate rate(1);
	while(ros::ok())
	{
		// execute the call backs, the buffers will be filled with n clouds
		ros::spinOnce();

		// node will update the point cloud
		node.update();

		// point cloud update rate
		rate.sleep();
	}

	return 0;
}


