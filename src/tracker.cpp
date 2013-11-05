#include "color_3D_tracker/tracker.h"
#include "color_3D_tracker/utils.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <numeric>
#include "opencv2/core/core.hpp"

/** \brief Tracker constructor
  * \param transport
  */
color_3D_tracker::Tracker::Tracker(const string transport) : StereoImageProcessor(transport)
{
  ROS_INFO_STREAM("[Tracker:] Instantiating the color_3D_tracker...");

  // Get all the params out!
  ros::NodeHandle nhp("~");
  nhp.param("stereo_frame_id", stereo_frame_id_, string("/stereo_down"));
  nhp.param("target_frame_id", target_frame_id_, string("/target"));
  nhp.param("closing_element_size", closing_element_size_, 0);
  nhp.param("opening_element_size", opening_element_size_, 1);
  nhp.param("binary_threshold", binary_threshold_, 80);
  nhp.param("min_value_threshold", min_value_threshold_, 110);
  nhp.param("min_blob_size", min_blob_size_, 8);
  nhp.param("max_blob_size", max_blob_size_, 200);
  nhp.param("epipolar_threshold", epipolar_threshold_, 3);
  nhp.param("mean_filter_size", mean_filter_size_, 1);
  nhp.param("max_tf_error", max_tf_error_, 0.1);
  nhp.param("trained_model_path", trained_model_path_, 
      color_3D_tracker::Utils::getPackageDir() + string("/etc/trained_model.yml"));
  nhp.param("show_debug", show_debug_, false);
  nhp.param("warning_on", warning_on_, false);
  nhp.param("listen_services", listen_services_, false);   

  ROS_INFO_STREAM("[Tracker:] color_3D_tracker Settings:" << endl <<
                  "  stereo_frame_id            = " << stereo_frame_id_ << endl <<
                  "  target_frame_id            = " << target_frame_id_ << endl <<
                  "  closing_element_size       = " << closing_element_size_ << endl <<
                  "  opening_element_size       = " << opening_element_size_ << endl <<
                  "  binary_threshold           = " << binary_threshold_ << endl <<
                  "  min_value_threshold        = " << min_value_threshold_ << endl <<
                  "  min_blob_size              = " << min_blob_size_ << endl <<
                  "  max_blob_size              = " << max_blob_size_ << endl <<
                  "  epipolar_threshold         = " << epipolar_threshold_ << endl <<
                  "  mean_filter_size           = " << mean_filter_size_ << endl <<
                  "  max_tf_error               = " << max_tf_error_ << endl <<
                  "  trained_model_path         = " << trained_model_path_ << endl <<
                  "  show_debug                 = " << show_debug_ << endl);

  // Read the trained model
  FileStorage fs(trained_model_path_, FileStorage::READ);
  fs["model_histogram"] >> trained_model_;
  fs.release();

  // Services to start or stop the model detection
  detect_service_ = nhp.advertiseService("detect", &Tracker::detectSrv, this);
  start_service_ = nhp.advertiseService("start_color_3D_detection", &Tracker::startDetectionSrv, this);
  stop_service_ = nhp.advertiseService("stop_color_3D_detection", &Tracker::stopDetectionSrv, this);

  if (listen_services_)
    do_detection_ = false;
  else
    do_detection_ = true;

  // Set the gui names
  tuning_gui_name_ = "Color 3D Tracker Tuning";

  // Initialize target tf
  camera_to_target_.setIdentity();
}

/** \brief Stereo Image Callback
  * \param l_image_msg message of the left image
  * \param r_image_msg message of the right image
  * \param l_info_msg information message of the left image
  * \param r_info_msg information message of the right image
  */
void color_3D_tracker::Tracker::stereoImageCallback(
  const sensor_msgs::ImageConstPtr     & l_image_msg,
  const sensor_msgs::ImageConstPtr     & r_image_msg,
  const sensor_msgs::CameraInfoConstPtr& l_info_msg,
  const sensor_msgs::CameraInfoConstPtr& r_info_msg)
{

  // Check if service is called or not
  if (listen_services_ && !(do_detection_ || toggle_detection_))
  {
    ROS_INFO("[Tracker:] Waiting for start service...");
    return;
  }

  // Images to opencv
  cv_bridge::CvImagePtr l_cv_image_ptr;
  cv_bridge::CvImagePtr r_cv_image_ptr;
  try
  {
    l_cv_image_ptr = cv_bridge::toCvCopy(l_image_msg,
                                         sensor_msgs::image_encodings::BGR8);
    r_cv_image_ptr = cv_bridge::toCvCopy(r_image_msg,
                                         sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("[Tracker:] cv_bridge exception: %s", e.what());
    return;
  }

  // For debuging
  processed_ = l_cv_image_ptr->image;

  // Get the camera model
  stereo_model_.fromCameraInfo(l_info_msg, r_info_msg);

  // Detect the color point in both images
  Point2d l_point_2d, r_point_2d;
  bool l_found = pointDetection(l_cv_image_ptr->image, l_point_2d, show_debug_);
  bool r_found = pointDetection(r_cv_image_ptr->image, r_point_2d, false);

  // Check number of points
  if(l_found && r_found)
  {
    // Get the 3D point
    Point3d point3d = triangulatePoints(l_point_2d, r_point_2d);

    // Put the point into a tf
    camera_to_target_.setIdentity();
    camera_to_target_.setOrigin(tf::Vector3(point3d.x, point3d.y, point3d.z));
  }

  // Publish the last pose
  tf_broadcaster_.sendTransform(
      tf::StampedTransform(camera_to_target_, l_image_msg->header.stamp,
      stereo_frame_id_, target_frame_id_));

  // Service
  toggle_detection_ = false;

  return;
}

/** \brief Detect the color points into the image
  * @return bool if point found
  * \param image where the color will be detected
  * \paramdetected color point
  * \param show debug or not
  */
bool color_3D_tracker::Tracker::pointDetection(Mat image, Point2d &point_2d, bool debug)
{
  // Compute backprojection
  Mat hsv_image;
  cvtColor(image, hsv_image, CV_BGR2HSV);
  Mat backprojection = color_3D_tracker::Utils::calculateBackprojection(hsv_image, trained_model_);

  // Used to draw the contours
  Mat contour_image(backprojection.size(), CV_8UC3, Scalar(0,0,0));
  cvtColor(backprojection, contour_image, CV_GRAY2RGB);

  // filter out noise
  if (mean_filter_size_ > 2 && mean_filter_size_ % 2 == 1)
  {
    medianBlur(backprojection, backprojection, mean_filter_size_);
  }

  // perform thresholding
  Mat binary;
  threshold(backprojection, binary, binary_threshold_, 255, CV_THRESH_BINARY);

  // morphological operations
  Mat binary_morphed = binary.clone();
  if (opening_element_size_ > 0)
  {
    Mat element = Mat::zeros(opening_element_size_, opening_element_size_, CV_8UC1);
    circle(element, Point(opening_element_size_ / 2, opening_element_size_ / 2), opening_element_size_ / 2, Scalar(255), -1);
    morphologyEx(binary_morphed, binary_morphed, MORPH_OPEN, element);
  }
  if (closing_element_size_ > 0)
  {
    Mat element = Mat::zeros(closing_element_size_, closing_element_size_, CV_8UC1);
    circle(element, Point(closing_element_size_ / 2, closing_element_size_ / 2), closing_element_size_ / 2, Scalar(255), -1);
    morphologyEx(binary_morphed, binary_morphed, MORPH_CLOSE, element);
  }  

  // create mask for invalid values
  vector<Mat> hsv_channels;
  split(hsv_image, hsv_channels);
  Mat value = hsv_channels[2];
  Mat min_value_mask;
  threshold(value, min_value_mask, min_value_threshold_, 255, CV_THRESH_BINARY);

  // mask out low values in binary image
  bitwise_and(min_value_mask, binary_morphed, binary_morphed);

  // Detect blobs in image
  vector< vector<Point> > contours;
  Mat contour_output = binary_morphed.clone();
  findContours(contour_output, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

  RotatedRect el;
  vector<Point> biggest_blob;

  // Check number of points detected
  if (contours.size() < 1)
  {
    if (debug)
    {
      vector< vector<Point> > mdl_contours;
      showTuning(contour_image, binary, binary_morphed, point_2d, mdl_contours);
    }
    return false;
  }

  // Delete too big and too small blobs
  vector< vector<Point> >::iterator iter = contours.begin();
  while (iter != contours.end())
  {
    if (iter->size() > (unsigned int)max_blob_size_ || 
        iter->size() < (unsigned int)min_blob_size_)
    {
      iter = contours.erase(iter);
    }
    else
    {
      ++iter;
    }
  }

  // Check number of points detected
  if (contours.size() < 1)
  {
    if (debug)
    {
      vector< vector<Point> > mdl_contours;
      showTuning(contour_image, binary, binary_morphed, point_2d, mdl_contours);
    }
    return false;
  }

  // If number of points is bigger than model points, return just the largest blob
  sort(contours.begin(), contours.end(), color_3D_tracker::Utils::sort_vectors_by_size);
  vector< vector<Point> > largest_points(contours.begin(), contours.begin() + 1);

  // Calculate mean point
  double u_mean = 0;
  double v_mean = 0;
  for (size_t j = 0; j < largest_points[0].size(); j++)
  {
    u_mean += largest_points[0][j].x;
    v_mean += largest_points[0][j].y;
  }
  u_mean /= largest_points[0].size();
  v_mean /= largest_points[0].size();
  Point mean_point(u_mean, v_mean);
  point_2d = mean_point;

  if (debug)
    showTuning(contour_image, binary, binary_morphed, point_2d, largest_points);

  return true;
}

/** \brief Triangulate the 3D point of the target
  * @return the 3D points of the target
  * \param point of the left image
  * \param point of the right image
  */
Point3d color_3D_tracker::Tracker::triangulatePoints(
  Point2d pl_2d, Point2d pr_2d)
{
  Point3d p3d;
  stereo_model_.projectDisparityTo3d(pl_2d, pl_2d.x-pr_2d.x, p3d);
  return p3d;
}

/** \brief Show the tuning gui
  * \param image where the contours are detected
  * \param binary image
  * \param binary morphed image
  * \param the model contours detected
  */
void color_3D_tracker::Tracker::showTuning(Mat contour_image, 
                                           Mat binary, 
                                           Mat binary_morphed, 
                                           Point2d point, 
                                           vector< vector<Point> > mdl_contours)
{
  Scalar color(0, 0, 255);
  drawContours(contour_image, mdl_contours, 0, color, 2);
  circle(contour_image, point, 15, color, 2);

  // Show images. First, convert to color
  Mat binary_color, binary_morphed_color;
  cvtColor(binary, binary_color, CV_GRAY2RGB);
  cvtColor(binary_morphed, binary_morphed_color, CV_GRAY2RGB);

  // Concatenate horizontaly the images
  Mat display_image(contour_image.size(), CV_8UC3);
  hconcat(contour_image, binary_color, display_image);
  hconcat(display_image, binary_morphed_color, display_image);

  // Create the window and the trackbars
  namedWindow(tuning_gui_name_, 0);
  setWindowProperty(tuning_gui_name_, CV_WND_PROP_ASPECTRATIO, CV_WINDOW_KEEPRATIO);
  createTrackbar("mean_filter_size", tuning_gui_name_, &mean_filter_size_, 255);
  createTrackbar("binary_threshold", tuning_gui_name_, &binary_threshold_, 255);
  createTrackbar("closing_element_size", tuning_gui_name_, &closing_element_size_, 255);
  createTrackbar("opening_element_size", tuning_gui_name_, &opening_element_size_, 255);
  createTrackbar("min_value_threshold", tuning_gui_name_, &min_value_threshold_, 255);
  createTrackbar("min_blob_size", tuning_gui_name_, &min_blob_size_, 5000);
  createTrackbar("max_blob_size", tuning_gui_name_, &max_blob_size_, 5000);

  // Get some reference values to position text in the image
  Size sz = contour_image.size();
  int W = sz.width; // image width
  //int H = sz.height; // image height
  int scale = 2; // text scale
  int thickness = 4; // text font thickness
  int clr = 255; // text colour (blue)
  int W0 = 10; // initial width 
  int H0 = 50; // initital height
  putText(display_image, "Contours", Point(W0,H0), FONT_HERSHEY_SIMPLEX, scale, clr, thickness);
  putText(display_image, "Binarized", Point(W+W0,H0), FONT_HERSHEY_SIMPLEX, scale, clr, thickness);
  putText(display_image, "Morph", Point(2*W+W0,H0), FONT_HERSHEY_SIMPLEX, scale, clr, thickness);
  imshow(tuning_gui_name_, display_image);
  waitKey(5);
}

/** \brief Services
  */
bool color_3D_tracker::Tracker::detectSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  camera_to_target_.setIdentity();
  do_detection_ = false;
  toggle_detection_ = true;
  ROS_INFO("Service Detect requested.");
  return true;
}
bool color_3D_tracker::Tracker::startDetectionSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  camera_to_target_.setIdentity();
  do_detection_ = true;
  ROS_INFO("Service Start Detection requested.");
  return true;
}
bool color_3D_tracker::Tracker::stopDetectionSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  do_detection_ = false;
  ROS_INFO("Service Stop Detection requested.");
  return true;
}