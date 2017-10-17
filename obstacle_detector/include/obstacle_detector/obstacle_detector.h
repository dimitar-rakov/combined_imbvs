#ifndef OBSTACLEDETECTOR_H
#define OBSTACLEDETECTOR_H

//ROS headers
#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include "obstacle_detector/SetObstacleTrajectory.h"

// TF
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"

//Standard Headers
#include <stdlib.h>

// Boost
#include <boost/thread/mutex.hpp>

//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>



static const bool USE_PCL_CB = true;

/**
 * @brief The ObstacleDetector class
 */
class ObstacleDetector
{
public:

  /**
     * @brief ObstacleDetector Default constructor
     */
  ObstacleDetector();

  /**
     * @brief init Initilializing of ObstacleDetector. It has to be called only once.
     * @param nh node handle
     * @return true if everithing pass normaly, false otherwise
     */
  bool init(ros::NodeHandle &nh);

  /**
     * @brief update Periodicaly called. All calculations related to this class are done whitin.
     * @param time Time from start
     * @param period Last update period
     */
  void update(const ros::Time &time, const ros::Duration &period);

  //
  /**
     * @brief publish All publishers are whitin. Data for publishing has to be prepared previously in update ()
     */
  void publish();



private:

  /// Node handle
  ros::NodeHandle nh_;

  /// Subscriber to the points
  ros::Subscriber sub_point_cloud_;

  /// Sevice for set the simulated obstacle trajectory
  ros::ServiceServer srv_set_obstacle_trajectory_ ;

  /// Publisher for reduced pointcloud
  ros::Publisher pub_output_cloud_;

  /// Publisher for markers related to obstacles objects
  ros::Publisher pub_markers_obstacles_objects_;

  /// Publisher for markers related to enviromental objects
  ros::Publisher pub_markers_filter_env_objects_;

  /// Publisher for markers related to robot objects
  ros::Publisher pub_markers_filter_robot_objects_;

  /// Publisher for markers related to robot objects fixed
  ros::Publisher pub_markers_filter_robot_objects_fixed_;

  /// Mutex for pointcloud callback
  boost::mutex  points_cb_mutex_;

  /// Radius of spheres used for approximation of robot body (in meters)
  double radius_sphere_robot_body_;

  /// Transform listener
  tf::TransformListener lr_;

  /// Container for all chain robot transformations (every TF is with respect to previous one)
  std::vector<tf::Transform> TFs_;

  /// Container for all robot transformations in wrt to defined on parameter server @par fixed_frame_
  std::vector<tf::Transform> TFs_fixed_;

  /// Marker array for approximation of the obstacles
  visualization_msgs::MarkerArray obstacles_objects_;

  /// Marker array for representing of enviromental objects
  visualization_msgs::MarkerArray filter_env_objects_;

  /// Marker array for representing of robot body with chain transformations
  visualization_msgs::MarkerArray filter_robot_objects_;

  /// Marker array for representing of robot body with fixed transformations
  visualization_msgs::MarkerArray filter_robot_objects_fixed_;

  /// Pointer related to ros pointcloud, used only in callback
  sensor_msgs::PointCloud2::Ptr cb_ros_cloud_ptr_;

  /// Pointer related to pcl pointcloud, used only in callback
  pcl::PointCloud<pcl::PointXYZ>::Ptr cb_cloud_ptr_;

  /// Pointer related to filtered ros pointcloud, used for publishing
  sensor_msgs::PointCloud2::Ptr filtered_ros_cloud_ptr_;

  /// Pointer related to filtered pcl pointcloud, used for publishing
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr_;

  /// Pointer related to pcl pointcloud, used only in update
  pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr_;

  /// Pointclouds topic (from parameter server)
  std::string points_topic_;

  /// Fixed frame in which are expresed the obstacles (from parameter server)
  std::string fixed_frame_;

  /// Name for the node (from parameter server)
  std::string base_name_;

  /// Container for robot links names (from parameter server)
  std::vector<std::string> tf_names_;

  /// Define the octree resolution used for obstacles interpolation
  double obs_octree_resolution_;

  /// Filtering  min amount of points whitin a voxel for real objects
  int min_voxel_points_;

  /// Flags for topics data. status -1 - not not received, status 0 - delayed,
  /// status 1 - receive in time and data ok, status 2 - receive in time and not valid data
  int points_status_;

  /// Safety Timers for topics
  ros::Time safety_ton_points_;

  /// Moving artifitial obstacle start position
  tf::Vector3 obst_start_position_;

  /// Moving artifitial obstacle final position
  tf::Vector3 obst_end_position_;

  ///  Moving artifitial obstacle time of receiving of start commmand
  double obs_start_time_;

  ///  Moving artifitial obstacle time for reaching of final position
  double obst_goal_time_;

  ///  Moving artifitial obstacle curent distance from start
  double obst_curr_distance_;

  ///  Enable new move for the artifitial obstacle
  bool new_move_;

  /**
   * @brief setObstacleTrajectory Service callback for set new trajectory for the artifitial obstacle
   * @param req
   * @param res
   * @return
   */
  bool setObstacleTrajectory(obstacle_detector::SetObstacleTrajectory::Request &req,
                             obstacle_detector::SetObstacleTrajectory::Response &res);

  /**
   * @brief pclPointcloudCB Callback for pcl pointcloud
   * @param cloud_msg Incomming message
   * @param dst_cloud_ptr Output destination pointer for pcl pointcloud
   * @param safety_ton Output destination  safety timer
   * @param points_status Output destination  points status
   */
  void pclPointcloudCB (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_msg,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr *dst_cloud_ptr,
                        ros::Time *safety_ton, int *points_status);

  /**
   * @brief rosPointcloudCB Callback for ros pointcloud
   * @param msg Incomming message
   * @param dst_cloud_ptr Output destination pointer for ros pointcloud
   * @param safety_ton Output destination  safety timer
   * @param points_status Output destination  points status
   */
  void rosPointcloudCB(const sensor_msgs::PointCloud2ConstPtr& msg,
                       sensor_msgs::PointCloud2Ptr *dst_cloud_ptr,
                       ros::Time *safety_ton, int *points_status);

  /**
   * @brief getTFs Get all robot transformations
   * @return
   */
  bool getTFs();

  /**
   * @brief buildRobotBodyFromSpheres Interpolate a robot body with sphere (no used anymore)
   */
  void buildRobotBodyFromSpheres();

  /**
   * @brief getFilterObjectsParameters Get filter object from an object on parameter server
   * @param obj Input object on parameter server
   * @param filter_objects Output destination for filter objects
   * @param ns namespace of the input object on parameter server
   */
  void getFilterObjectsParameters(XmlRpc::XmlRpcValue &obj,
                                  visualization_msgs::MarkerArray &filter_objects,
                                  const std::string &ns);

  /**
   * @brief filterBoxOut Input filter out a box defined by @par filter_objects from input pcl pointcloud
   * @param filter_object Input filter object marker, which parameter are used by filtration
   * @param in_cloud_ptr  Input pointer to input pointcloud
   * @param filtered_cloud_ptr Output pointer to filtered pointcloud
   */
  void filterBoxOut(const visualization_msgs::Marker &filter_object,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud_ptr,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud_ptr);

  /**
   * @brief filterSphereOut  Input filter out a sphere defined by @par filter_objects from input pcl pointcloud
   * @param filter_object Input filter object marker, which parameter are used by filtration
   * @param in_cloud_ptr Input pointer to input pointcloud
   * @param filtered_cloud_ptr Output pointer to filtered pointcloud
   */
  void filterSphereOut(const visualization_msgs::Marker &filter_object,
                       const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud_ptr,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud_ptr);


  /**
   * @brief simMovingObstacle
   * @param period Input update period
   */
  void simMovingObstacle(const ros::Duration &period);

  /**
   * @brief detectObstaclesAsBox Approximate obstacles as a single box
   */
  void detectObstaclesAsBox();

  /**
   * @brief detectOctreeVoxels Approximate obstacle whit octree voxels
   */
  void detectOctreeVoxels();
};

#endif // OBSTACLEDETECTOR_H
