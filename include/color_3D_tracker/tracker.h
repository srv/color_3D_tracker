/**
 * @file
 * @brief color_3D_tracker presentation.
 */

#ifndef TRACKER_H
#define TRACKER_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_geometry/stereo_camera_model.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>
#include "color_3D_tracker/stereo_processor.h"

using namespace std;
using namespace cv;

namespace color_3D_tracker
{

class Tracker : public StereoImageProcessor
{

public:
  // Constructor
  Tracker(const string transport);

  // Image points detection
  bool pointDetection(
    Mat image, Point2d &point_2d, bool debug);

private:
  // Node parameters
  string stereo_frame_id_;
  string target_frame_id_;
  image_transport::Publisher image_pub_;
  ros::ServiceServer detect_service_;
  ros::ServiceServer start_service_;
  ros::ServiceServer stop_service_;

  // Tracker parameters
  int closing_element_size_;
  int opening_element_size_;
  int binary_threshold_;
  int min_blob_size_;
  int max_blob_size_;
  int epipolar_threshold_;
  int mean_filter_size_;
  int min_value_threshold_;
  double max_tf_error_;
  string trained_model_path_;
  string tuning_gui_name_;
  MatND trained_model_;
  bool show_debug_;
  bool warning_on_;
  bool listen_services_;
  bool do_detection_;
  bool toggle_detection_; // just one detection is required

  // TF
  tf::TransformBroadcaster tf_broadcaster_;           //!> Transform publisher
  tf::Transform camera_to_target_;                    //!> Camera to target transformation

  Mat processed_;                                     //!> Processed image
  image_geometry::StereoCameraModel stereo_model_;    //!> Camera model to compute the 3d world points

  void stereoImageCallback(
      const sensor_msgs::ImageConstPtr& l_image_msg,
      const sensor_msgs::ImageConstPtr& r_image_msg,
      const sensor_msgs::CameraInfoConstPtr& l_info_msg,
      const sensor_msgs::CameraInfoConstPtr& r_info_msg); //!> Image callback

  static void staticMouseCallback(int event, int x, 
    int y, int flags, void* param);                       //!> Mouse callback interface
  void mouseCallback( int event, int x, int y, 
    int flags, void* param);                              //!> Mouse interface
  
  Point3d triangulatePoints(Point2d pl_2d, Point2d pr_2d);//!> Triangulate the 3D point

  void showTuning(Mat contour_image, 
                  Mat binary, 
                  Mat binary_morphed, 
                  Point2d points, 
                  vector< vector<Point> > mdl_contours);  //!> Function to show the tuning GUI

  bool detectSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
  bool startDetectionSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
  bool stopDetectionSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
};

} // namespace

#endif // TRACKER_H
