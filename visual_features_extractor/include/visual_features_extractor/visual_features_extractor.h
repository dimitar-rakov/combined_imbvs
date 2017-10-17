#ifndef VISUAL_FEATURES_EXTRACTOR_H
#define VISUAL_FEATURES_EXTRACTOR_H

//ROS headers
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>             //interface between ROS and OpenCV
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <tf_conversions/tf_eigen.h>
#include "visual_features_extractor/VisFeature.h"
#include "visual_features_extractor/SetTF.h"
#include "visual_features_extractor/SetDepth.h"
#include "sensor_msgs/JointState.h"

//OpenCV headers
#include <opencv2/core/core.hpp>

//Pthread Headers
#include <boost/thread/mutex.hpp>

//Eigen
#include <eigen3/Eigen/Dense>

//Aruco headers
#include <aruco/aruco.h>


/**
 * @brief The VisualFeaturesExtractor class
 */

class VisualFeaturesExtractor
{
public:

  /**
     * @brief VisualFeaturesExtractor Default constructor
  */
  VisualFeaturesExtractor();

  /**
   * @brief init Initilializing of VisualFeaturesExtractor. It has to be called only once before update().
   * @param nh Node handle
   * @return true if everithing pass normaly, false otherwise
   */
  bool init(ros::NodeHandle &nh);

  /**
   * @brief update Periodicaly called. All calculations related to this class are done whitin.
   * @param time Time from start
   * @param period Last update period
   */
  void update(const ros::Time &time, const ros::Duration &period);

private:
  /// Node handle
  ros::NodeHandle nh_;

  /// Service to set transformation for simulated features with respect to world.
  ros::ServiceServer srv_set_sim_features_tf_ ;

  ///  Service to set transformation for desired features with respect to image.
  ros::ServiceServer srv_set_des_features_tf_ ;

  ///  Service to set desired template image.
  ros::ServiceServer srv_set_des_template_ ;

  ///  Image transport for camera raw images
  image_transport::ImageTransport it_;

  ///  Image subscriber for camera raw images
  image_transport::Subscriber sub_img_transport_;

  /// Publisher for calculated features vectrors and interaction matrices
  ros::Publisher  pub_vis_data_;

  /// Message for visual data publisher
  visual_features_extractor::VisFeature visual_feature_msg_;

  /// Container for camera info
  sensor_msgs::CameraInfo cam_param_;

  /// Mutex for image callback
  boost::mutex  image_cb_mutex_;

  /// Transform broadcaster for all simulated features transform
  tf::TransformBroadcaster br_;

  /// Transform listener
  tf::TransformListener lr_;

  /// Container for all transformations related to simulated features
  std::vector <tf::StampedTransform> allTFwff_;

  /// Container for all transformations related to desired features
  std::vector <tf::Transform> allTFcdf_;

  /// Transform for simulated features wrt to world
  tf::Transform TFwsf_;

  /// Transform for desired features wrt to camera
  tf::Transform TFcdf_;

  /// Transform for end effector wrt to world
  tf::Transform TFwe_;

  /// Transform for world wrt to camera
  tf::Transform TFcw_;

  /// Transform for camera wrt to end effector
  tf::Transform TFec_;

  /// Enable simulated features  (from parameter server)
  bool using_sim_features_;

  ///  Enable extended features (image moment based features defined by @var extended_features_var_ )  (from parameter server)
  bool using_extended_features_;

  /// Use symetrical features, otherwise non symmetrical (from parameter server)
  bool using_symmetrical_features_;

  /// Use colored features, otherwise are used aruco marker (from parameter server)
  bool using_colored_blobs_;

  /// Topic for raw images (from parametere server)
  std::string raw_images_topic_;

  /// Name for the node (from parametere server)
  std::string base_name_;

  /// Name for the camera (from parametere server)
  std::string camera_name_;

  /// Container for names of extracted features (from parameter server)
  std::vector <std::string> features_names_;

  /// Variant for features vector (from parameter server)
  double extended_features_var_;

  /// Number of extracted features
  int num_extracted_features_;

  /// Min threshold for  contour area  (from parameter server)
  double min_contour_area_threshold_;

  /// Hue limits for coloured blobs  (from parameter server)
  std::vector <int> hue_min_limits_, hue_max_limits_;

  /// ids for aruco markers  (from parameter server)
  std::vector <int> arucos_id;

  /// Sizes for aruco markers  (from parameter server)
  std::vector <double> arucos_size;

  /** @name Raw and central moments
   * @brief  Low grade raw and central moments
   */
  ///@{
  double xg_, yg_, m00_, m01_, m10_, m11_ , m20_, m02_, m12_;
  double m21_,m22_, m03_, m30_, m04_, m13_, m31_, m40_, mu11_, mu12_;
  double mu21_, mu02_, mu20_, mu03_, mu30_ ,mu02des_ , mu20des_;
  ///@}


  /** @name Variable related to specific moments
   * @brief  variable related to specific moments
   */
  ///@{
  double I1_, I2_, I3_, px_, py_, sx_, sy_, alpha_;
  double c1_, c2_, c3_, s1_, s2_, s3_, k_, an_, a_ ,a_des_;
  double t1_ ,t2_, P2_ ,P3_ , delta_, K_;
  ///@}

  /// distance to measured feature wrt to camera origin
  double Z_;

  /// distance to desired feature wrt to camera origin
  double Z_des_;

  /// Calculated angle between measured extracted features and horizontal image axis
  double msr_angle_;

  /// Calculated angle between desired extracted features and horizontal image axis
  double des_angle_;

  /// index of fartest feature from first feature
  int idx_max_dist_;

  /// Container all features' x coordinates
  Eigen::VectorXd x_;

  /// Container all features' y coordinates
  Eigen::VectorXd y_;

  /// Measured features vector
  Eigen::VectorXd s_msr_;

  /// Measured features interaction matrix
  Eigen::MatrixXd Lhat_msr_;

  /// Desired features vector
  Eigen::VectorXd s_des_;

  /// Desired features interaction matrix
  Eigen::MatrixXd Lhat_des_;

  /// Image pointer used only in callback
  cv_bridge::CvImagePtr cb_images_ptr_;

  /// Image pointer used in update
  cv_bridge::CvImagePtr in_images_ptr_;

  /// Container for rectifyied image
  cv::Mat rect_image_;

  /// Container for masked image, used by coloured features extraction
  cv::Mat mask_image_;

  /// Container for  image, used show image  function
  cv::Mat drawing_image_;

  /// Aruco camera parameters
  aruco::CameraParameters aruco_cam_params_;

  /// Work features coordinates in image space
  std::vector< cv::Point2d> work_features_coord_;

  /// Simulated features coordinates in image space
  std::vector< cv::Point2d>sim_features_coord_;

  /// Desired features coordinates in image space
  std::vector< cv::Point2d>des_features_coord_;

  /// Topics' data status : [-1] - not not received, [0] - delayed, status [1] - receive in time and data ok
  int image_status_;

  /// Safety timers for topics
  ros::Time safety_ton_image_;

  /**
   * @brief setTfSimFeatureWrtWorld Service callback for setting a transform of the simulated features wrt to world
   * @param req
   * @param res
   * @return
   */
  bool setTfSimFeatureWrtWorld (visual_features_extractor::SetTF::Request &req,
                                visual_features_extractor::SetTF::Response &res);

  /**
   * @brief setTfDesFeatureWrtCamera Service callback for setting a transform of the desired features wrt to camera
   * @param req
   * @param res
   * @return
   */
  bool setTfDesFeatureWrtCamera(visual_features_extractor::SetTF::Request &req,
                                visual_features_extractor::SetTF::Response &res);

  /**
   * @brief setDesiredTemplate Service callback for setting a desired templates
   * @param req
   * @param res
   * @return
   */
  bool setDesiredTemplate(visual_features_extractor::SetDepth::Request &req,
                          visual_features_extractor::SetDepth::Response &res);


  /**
   * @brief getTFs Get all transforms, needed for algorithms
   * @return
   */
  bool getTFs();

  /**
   * @brief getCameraParameter Get camera parameter from parameter server
   * @param nh Input node handle
   * @param camera_name  Input camera name
   * @param dst_camera_param Output destination container for camera parameters
   * @return
   */
  bool getCameraParameter(const ros::NodeHandle &nh,
                          const std::string &camera_name,
                          sensor_msgs::CameraInfo &dst_camera_param);

  /**
   * @brief findBlobsContours   Find contours of coloured blobs
   * @param srs_image Input source image used for contour extraction
   * @param dst_contours Output destination container for found contours
   * @param dst_found_features_names Output destination container for found features names
   */
  void findBlobsContours(const cv::Mat &srs_image,
                         std::vector<std::vector<cv::Point> >  &dst_contours,
                         std::vector<std::string> &dst_found_features_names);

  /**
   * @brief findArucoMarkers Find aruco markers
   * @param srs_image Input image used for aruco extraction
   * @param dst_markers Output destination container for found markers
   * @param founded_features_names Output destination container for found features names
   */
  void findArucoMarkers(const cv::Mat &srs_image,
                        std::vector<aruco::Marker>  &dst_markers,
                        std::vector<std::string> &founded_features_names);

  /**
   * @brief showAll Visualisation of found contours or aruco markers
   * @param srs_image Input image used for visualization
   * @param src_contours Input countours
   * @param srs_markers Input aruco markers
   * @param src_coord Input features coordinates
   * @param founded_features_names Output destination container for found features names
   */
  void showAll(const cv::Mat &srs_image, const std::vector<std::vector<cv::Point> > &src_contours,
               const std::vector<aruco::Marker>  &srs_markers,
               const std::vector<cv::Point2d> &src_coord,
               const std::vector<std::string> &founded_features_names);


  /**
   * @brief getBestNContours Order the countours by size and take the largest
   * @param srs_contours  Input countours
   * @param contour_area_treshold Input minimum area threshold
   * @return
   */
  std::vector<std::vector<cv::Point> > getBestNContours(
      const std::vector<std::vector<cv::Point> > &srs_contours,
      int contour_area_treshold);

  /**
   * @brief calcFeaturesParameters Calculate image moments for given number of features and build matrices s, L,
   * @param in_features_coord Input feautures coourdinates
   * @return
   */
  bool calcFeaturesParameters(const std::vector<cv::Point2d> &in_features_coord);

  /**
   * @brief imageCB Callback function to images raw topic
   * @param msg Incoming message
   * @param dst_image_ptr Output destination  to image pointer
   * @param safety_ton Output destination safety timer
   * @param image_status Output destination image status
   */
  void imageCB(const sensor_msgs::ImageConstPtr& msg,
               cv_bridge::CvImagePtr *dst_image_ptr,
               ros::Time *safety_ton, int *image_status);

  /**
   * @brief calcSpatialMoment Calculate a central moment m_ij
   * @param coord Input features coordinates
   * @param i Input i-th order of the moment
   * @param j Input j-th order of the moment
   * @return
   */
  double calcSpatialMoment(const std::vector<cv::Point2d> &coord, const int &i, const int &j);

  //
  /**
   * @brief calcCentralMoment Calculate a spatial moment mu_ij
   * @param coord Input features coordinates
   * @param i Input i-th order of the moment
   * @param j Input j-th order of the moment
   * @return
   */
  double calcCentralMoment(const std::vector<cv::Point2d> &coord , const int &i, const int &j);


  /**
   * @brief calcSimple Calculation of classical feature vector and
   * interaction matrix based on features' image coordinates
   * @param s Output destination to features vector
   * @param L Output destination to interaction matrix
   */
  void calcSimple(Eigen::VectorXd &s, Eigen::MatrixXd &L);

  /**
   * @brief calcExtendedAsymmetricalV1_0 Calculation of extended feature vector and
   * interaction matrix based on variant 1.0 features' moments for the case with asymmetrical features
   * @param s Output destination to features vector
   * @param L_asym Output destination to interaction matrix
   */
  void calcExtendedAsymmetricalV1_0(Eigen::VectorXd &s, Eigen::MatrixXd &L_asym);

  /**
   * @brief calcExtendedSymmetricalV1_0 Calculation of extended feature vector and
   * interaction matrix based on variant 1.0 features' moments for the case with symmetrical features
   * @param s Output destination to features vector
   * @param L_sym Output destination to interaction matrix
   */
  void calcExtendedSymmetricalV1_0(Eigen::VectorXd &s, Eigen::MatrixXd &L_sym);

  /**
   * @brief calcExtendedAsymmetricalV1_1 Calculation of extended feature vector and
   * interaction matrix based on variant 1.1 features' moments for the case with asymmetrical features
   * @param s Output destination to features vector
   * @param L_asym Output destination to interaction matrix
   */
  void calcExtendedAsymmetricalV1_1(Eigen::VectorXd &s, Eigen::MatrixXd &L_asym);

  /**
   * @brief calcExtendedSymmetricalV1_1 Calculation of extended feature vector and
   * interaction matrix based on variant 1.1 features' moments for the case with symmetrical features
   * @param s Output destination to features vector
   * @param L_sym Output destination to interaction matrix
   */
  void calcExtendedSymmetricalV1_1(Eigen::VectorXd &s, Eigen::MatrixXd &L_sym);

  /**
   * @brief calcExtendedAsymmetricalV1_2 Calculation of extended feature vector and
   * interaction matrix  based on variant 1.2 features' moments for the case with asymmetrical features
   * @param s Output destination to features vector
   * @param L_asym Output destination to interaction matrix
   */
  void calcExtendedAsymmetricalV1_2(Eigen::VectorXd &s, Eigen::MatrixXd &L_asym);

  /**
   * @brief calcExtendedSymmetricalV1_2 Calculation of extended feature vector and
   * interaction matrix based on variant 1.2 features' moments for the case with symmetrical features
   * @param s Output destination to features vector
   * @param L_sym  Output destination to interaction matrix
   */
  void calcExtendedSymmetricalV1_2(Eigen::VectorXd &s, Eigen::MatrixXd &L_sym);

  /**
   * @brief calcExtendedAsymmetricalV1_3 Calculation of extended feature vector and
   * interaction matrix based on variant 1.3 features' moments for the case with asymmetrical features
   * @param s Output destination to features vector
   * @param L_asym Output destination to interaction matrix
   */
  void calcExtendedAsymmetricalV1_3(Eigen::VectorXd &s, Eigen::MatrixXd &L_asym);

  /**
   * @brief calcExtendedSymmetricalV1_3 Calculation of extended feature vector and
   * interaction matrix based on variant 1.3 features' moments for the case with symmetrical features
   * @param s Output destination to features vector
   * @param L_sym Output destination to interaction matrix
   */
  void calcExtendedSymmetricalV1_3(Eigen::VectorXd &s, Eigen::MatrixXd &L_sym);

  /**
   * @brief simVisualFeaturesTFs Calculate all simulated features
   * transforms wrt to world (Euclidean coordinates)
   */
  void simVisualFeaturesTFs();

  /**
   * @brief desVisualFeaturesTFs Calculate all desired features transforms wrt to camera (pixel coordinates)
   */
  void desVisualFeaturesTFs();

  /**
   * @brief calcFeaturesImageCoord Calculate image coordinates of extracted features
   */
  void calcFeaturesImageCoord();

};

#endif // VIS_FEATURE_EXTRACT_H
