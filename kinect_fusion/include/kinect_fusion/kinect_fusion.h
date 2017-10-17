#ifndef KINECT_FUSION_H
#define KINECT_FUSION_H

//ROS headers
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


// kinect_fusion  specific msg and srv
#include "kinect_fusion/SetTask.h"

// Open CV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/aruco.h>

//Standard Headers
#include <stdlib.h>
#include <boost/thread/mutex.hpp>


#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

static const int QUEUE_SIZE =3;


/**
 * @brief The KinectFusion class
 */
class KinectFusion
{
public:
  typedef sensor_msgs::PointCloud2 PC2;
  typedef message_filters::sync_policies::ApproximateTime<PC2, PC2, PC2, PC2, PC2, PC2> sync_policy6;

  /**
     * @brief KinectFusion Default constructor
     */
  KinectFusion();
  /**
     * @brief init Initilializing of KinectFusion. It has to be called only once before update().
     * @param nh Input node handle
     * @return true if everithing pass normaly, false otherwise
     */
  bool init(ros::NodeHandle &nh);

  /**
     * @brief update Periodicaly called. All calculations related to the class are done whitin.
     * @param time Input time from start
     * @param period Input last update period
     */
  void update(const ros::Time& time, const ros::Duration& period);

private:

  /// Node handle
  ros::NodeHandle nh_;

  /// Subscribers to the points
  std::vector<ros::Subscriber> subs_points_;

  /// Subscribers to the cameras info
  std::vector<ros::Subscriber> subs_cam_info_;

  /// Container for message filters
  std::vector<boost::shared_ptr<message_filters::Subscriber<PC2> > > msg_filters_;

  /// Shared pointer for synchronizer
  boost::shared_ptr<message_filters::Synchronizer<sync_policy6> > ts_a6_ptr_;

  /// Publisher for fussed pointcloud
  ros::Publisher pub_fussed_points;

  /// Publisher for transformed poincloud of sensor 1 \todo {Remove in future, since it is only for test purposes}
  ros::Publisher pub_transformed_points1;

  /// Publisher for transformed poincloud of sensor 2 \todo {Remove in future, since it is only for test purposes}
  ros::Publisher pub_transformed_points2;

  /// Container for all transformations from sensors' frames to world
  std::vector< tf::Transform >  TFs_w_c;

  /// Container for all transformations from aruco marker frame to sensors' frames
  std::vector<tf::Transform> TFs_a_c_;

  /// Transform listener
  tf::TransformListener lr_;

  /// Container for subscribers related to raw images
  std::vector<image_transport::Subscriber> subs_image_raw_;

  /// Container forimage transports related to raw images
  std::vector<image_transport::ImageTransport> images_tran_;

  /// Container for image pointers, used only in callback
  std::vector<cv_bridge::CvImagePtr> cb_images_ptr_;

  /// Container for image pointers, used in update
  std::vector<cv_bridge::CvImagePtr> in_images_ptr_;

  /// Container for pointers related to camera infos, used only in callback
  std::vector<sensor_msgs::CameraInfo::Ptr> cb_cam_info_ptr_;

  /// Container for pointers related to camera infos, used in update
  std::vector<sensor_msgs::CameraInfo::Ptr> in_cam_info_ptr_;

  /// Container for pointers related to pointclouds, used only in callback
  std::vector<sensor_msgs::PointCloud2::Ptr> cb_clouds_ptr_;

  /// Container for pointers related to pointclouds, used in update
  std::vector<sensor_msgs::PointCloud2::Ptr> in_clouds_ptr_;

  /// Container for pointers of  transformed pointclouds
  std::vector<sensor_msgs::PointCloud2::Ptr> transf_clouds_ptr_;

  /// Pointer for fused pointcloud
  sensor_msgs::PointCloud2::Ptr fused_cloud_ptr_;

   /// Name for the node (from parameter server)
  std::string base_name_;

  /// Container for raw images topics (from parameter server)
  std::vector<std::string> raw_images_topics_;

  /// Container for pointclouds topics (from parameter server)
  std::vector<std::string> point_topics_;

  /// Container for cameras info topics (from parameter server)
  std::vector<std::string> cam_info_topics_;

  /// Enable using of aruco (from parameter server).
  /// This start also all image subcriber and all related image operations.
  /// Therefore, it has to be used only for calculation of extrinsic parameters
  bool using_aruco_ ;

  /// Size of aruco marker in meters  (from parameter server)
  double aruco_marker_size_;

  /// Aruco marler id (not used) (from parameter server)
  int aruco_marker_id_;

  /// Containers for flags related to raw images topics data. status -1 - not not received, status 0 - delayed,
  /// status 1 - receive in time and data ok, status 2 - receive in time and not valid data,
  std::vector<int> image_status_;

  /// Containers for flags related to cameras info topics data. status -1 - not not received, status 0 - delayed,
  /// status 1 - receive in time and data ok, status 2 - receive in time and not valid data,
  std::vector<int> cam_info_status_;

  /// Flag related to pointcloud topics data. status -1 - not not received, status 0 - delayed,
  /// status 1 - receive in time and data ok, status 2 - receive in time and not valid data,
  int points_status_;

  /// Safety timers for raw images topics
  std::vector<ros::Time> safety_tons_images_;

  /// Safety timers for cameras info topics
  std::vector<ros::Time> safety_tons_cam_info_;

  /// Safety timer for sinchronizer update
  ros::Time safety_tons_points_;

  /// Mutex for image callbacks
  boost::mutex image_cb_mutex_;

  /// Mutex for synchronizer callback
  boost::mutex sync_cb_mutex_;

  /// Mutex for cameras info callbacks
  boost::mutex cam_info_cb_mutex_;



  /**
   * @brief markerDetect Detect aruco marker in scene
   * @param srs_image Input source image used for aruco extraction
   * @param cam_info_ptr Input pointer to camera info
   * @param dstTF Output destination transform for found markers
   * @param marker_id Input aruco marker id
   * @param marker_size Input aruco marker size
   * @param windows_name Input window name for imshow()
   */
  void markerDetect(const cv:: Mat& srs_image, const sensor_msgs::CameraInfoPtr &cam_info_ptr,
                    tf::Transform &dstTF, int marker_id, double marker_size, std::string windows_name );

  /**
   * @brief imageCB Callback function to images raw topic
   * @param msg Incoming message
   * @param dst_image_ptr  Output destination  to image pointer
   * @param safety_ton Output destination  safety timer
   * @param image_status Output destination  image status
   */
  void imageCB(const sensor_msgs::ImageConstPtr& msg, cv_bridge::CvImagePtr *dst_image_ptr,
               ros::Time *safety_ton, int *image_status);

  /**
   * @brief cameraInfoCB Callback function to camera info
   * @param msg Incoming message
   * @param dst_cam_info_ptr  Output destination to camera info pointer
   * @param dst_safety_ton Output destination safety timer
   * @param dst_status Output destination safety timer
   */
  void cameraInfoCB(const sensor_msgs::CameraInfoConstPtr& msg,
                    sensor_msgs::CameraInfoPtr* dst_cam_info_ptr,
                    ros::Time *dst_safety_ton, int *dst_status);

  /**
   * @brief syncPointcloudsCB Synchronized callback function to all point raw topic
   * @param msg1 Incoming pointcloud 1 message
   * @param msg2 Incoming pointcloud 2 message
   * @param msg3 Incoming pointcloud 3 message
   * @param msg4 Incoming pointcloud 4 message
   * @param msg5 Incoming pointcloud 5 message
   * @param msg6 Incoming pointcloud 6 message
   */
  void syncPointcloudsCB( const sensor_msgs::PointCloud2ConstPtr& msg1, const sensor_msgs::PointCloud2ConstPtr& msg2,
                          const sensor_msgs::PointCloud2ConstPtr& msg3, const sensor_msgs::PointCloud2ConstPtr& msg4,
                          const sensor_msgs::PointCloud2ConstPtr& msg5, const sensor_msgs::PointCloud2ConstPtr& msg6);

  /**
   * @brief pointcloudsFusion All fusion calculations
   * @param in_clouds_ptr Container for all input pointlcoud pointers
   */
  void pointcloudsFusion(std::vector<sensor_msgs::PointCloud2::Ptr>  in_clouds_ptrs);
};

#endif // KINECT_FUSION_H
