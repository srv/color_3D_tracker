#include "color_circle_tracker/tracker.h"
#include "color_circle_tracker/utils.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <numeric>
#include "opencv2/core/core.hpp"

/** \brief Tracker constructor
  * \param transport
  */
color_circle_tracker::Tracker::Tracker(cv::MatND trained_model, 
                                std::vector<cv::Point3d> color_circle_model, 
                                image_geometry::StereoCameraModel stereo_model, 
                                int epipolar_width_threshold) : StereoImageProcessor()
{
  // Setup the basic parameters
  trained_model_ = trained_model;
  color_circle_model_points_ = color_circle_model;
  stereo_model_ = stereo_model;
  epipolar_width_threshold_ = epipolar_width_threshold;
}
color_circle_tracker::Tracker::Tracker(const std::string transport) : StereoImageProcessor(transport)
{
  ROS_INFO_STREAM("[Tracker:] Instantiating the color_circle Tracker...");

  // Get all the params out!
  ros::NodeHandle nhp("~");
  nhp.param("stereo_frame_id", stereo_frame_id_, std::string("/stereo_down"));
  nhp.param("color_circle_frame_id", color_circle_frame_id_, std::string("/color_circle"));
  nhp.param("closing_element_size", closing_element_size_, 0);
  nhp.param("opening_element_size", opening_element_size_, 1);
  nhp.param("binary_threshold", binary_threshold_, 80);
  nhp.param("min_value_threshold", min_value_threshold_, 110);
  nhp.param("min_blob_size", min_blob_size_, 8);
  nhp.param("max_blob_size", max_blob_size_, 200);
  nhp.param("epipolar_width_threshold", epipolar_width_threshold_, 3);
  nhp.param("mean_filter_size", mean_filter_size_, 1);
  nhp.param("max_tf_error", max_tf_error_, 0.1);
  nhp.param("max_rot_diff", max_rot_diff_, 0.175);
  nhp.param("max_trans_diff", max_trans_diff_, 0.05);
  nhp.param("trained_model_path", trained_model_path_, 
      color_circle_tracker::Utils::getPackageDir() + std::string("/etc/trained_model.yml"));
  nhp.param("show_debug", show_debug_, false);
  nhp.param("warning_on", warning_on_, false);
  nhp.param("tf_filter_size", tf_filter_size_, 0);
  nhp.param("listen_services", listen_services_, false);   

  ROS_INFO_STREAM("[Tracker:] color_circle Tracker Settings:" << std::endl <<
                  "  stereo_frame_id            = " << stereo_frame_id_ << std::endl <<
                  "  color_circle_frame_id             = " << color_circle_frame_id_ << std::endl <<
                  "  closing_element_size       = " << closing_element_size_ << std::endl <<
                  "  opening_element_size       = " << opening_element_size_ << std::endl <<
                  "  binary_threshold           = " << binary_threshold_ << std::endl <<
                  "  min_value_threshold        = " << min_value_threshold_ << std::endl <<
                  "  min_blob_size              = " << min_blob_size_ << std::endl <<
                  "  max_blob_size              = " << max_blob_size_ << std::endl <<
                  "  epipolar_width_threshold   = " << epipolar_width_threshold_ << std::endl <<
                  "  mean_filter_size           = " << mean_filter_size_ << std::endl <<
                  "  max_tf_error               = " << max_tf_error_ << std::endl <<
                  "  max_rot_diff               = " << max_rot_diff_ << std::endl <<
                  "  max_trans_diff             = " << max_trans_diff_ << std::endl <<
                  "  tf_filter_size             = " << tf_filter_size_ << std::endl <<
                  "  trained_model_path         = " << trained_model_path_ << std::endl);

  // Read the trained model
  cv::FileStorage fs(trained_model_path_, cv::FileStorage::READ);
  fs["model_histogram"] >> trained_model_;
  fs.release();

  // Services to start or stop the color_circle detection
  detect_service_ = nhp.advertiseService("detect", &Tracker::detectSrv, this);
  start_service_ = nhp.advertiseService("start_color_circle_detection", &Tracker::startDetectionSrv, this);
  stop_service_ = nhp.advertiseService("stop_color_circle_detection", &Tracker::stopDetectionSrv, this);

  if (listen_services_)
    do_detection_ = false;
  else
    do_detection_ = true;

  // Set the gui names
  tuning_gui_name_ = "color_circle Tracker Tuning";
}

/** \brief Show the current parameter set for console.
  */
void color_circle_tracker::Tracker::showParameterSet()
{
  ROS_INFO_STREAM("[Tracker:] Parameter set: [" <<
                  mean_filter_size_ << ", " <<
                  binary_threshold_ << ", " <<
                  closing_element_size_ << ", " <<
                  opening_element_size_ << ", " <<
                  min_value_threshold_ << ", " <<
                  min_blob_size_ << ", " <<
                  max_blob_size_ << "]");
}

/** \brief Set a parameter
  * \param name of the parameter
  * \param value of the parameter
  */
void color_circle_tracker::Tracker::setParameter(std::string param_name, int param_value)
{
  if (param_name == "mean_filter_size") 
    mean_filter_size_ = param_value;
  else if (param_name == "binary_threshold") 
    binary_threshold_ = param_value;
  else if (param_name == "min_value_threshold")
    min_value_threshold_ = param_value;
  else if (param_name == "closing_element_size")
    closing_element_size_ = param_value;
  else if (param_name == "opening_element_size") 
    opening_element_size_ = param_value;
  else if (param_name == "min_blob_size")
    min_blob_size_ = param_value;
  else if (param_name == "max_blob_size")
    max_blob_size_ = param_value;
}

/** \brief Stereo Image Callback
  * \param l_image_msg message of the left image
  * \param r_image_msg message of the right image
  * \param l_info_msg information message of the left image
  * \param r_info_msg information message of the right image
  */
void color_circle_tracker::Tracker::stereoImageCallback(
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

  // Detect color_circle in both images
  std::vector<int> l_contours_size, r_contours_size;
  cv::RotatedRect el_l = circleDetection(l_cv_image_ptr->image, show_debug_, l_contours_size);
  cv::RotatedRect el_r = circleDetection(r_cv_image_ptr->image, false, r_contours_size);

  cv::Mat debug_image = l_cv_image_ptr->image.clone();
  cv::ellipse(debug_image, el_l, cv::Scalar(255,0,0), 2, 8 );
  cv::ellipse(debug_image, el_r, cv::Scalar(255,0,0), 2, 8 );

  

  cv::namedWindow("kk");
  cv::imshow("kk",debug_image);
  cv::waitKey(3);

  toggle_detection_ = false;

  return;
}

/** \brief Detect the color_circle into the image
  * @return vector with the detected color_circle points
  * \param image where the color_circle will be detected
  */
cv::RotatedRect color_circle_tracker::Tracker::circleDetection(cv::Mat image, bool debug)
{
  std::vector<int> contours_size;
  return circleDetection(image, debug, contours_size);
}
cv::RotatedRect color_circle_tracker::Tracker::circleDetection(cv::Mat image, bool debug, 
                                                                std::vector<int> &contours_size)
{
  // Initialize output
  std::vector<cv::Point2d> points;

  // Compute backprojection
  cv::Mat hsv_image;
  cv::cvtColor(image, hsv_image, CV_BGR2HSV);
  cv::Mat backprojection = color_circle_tracker::Utils::calculateBackprojection(hsv_image, trained_model_);

  // Used to draw the contours
  cv::Mat contour_image(backprojection.size(), CV_8UC3, cv::Scalar(0,0,0));
  cv::cvtColor(backprojection, contour_image, CV_GRAY2RGB);
  cv::Scalar color(0, 0, 255);

  // filter out noise
  if (mean_filter_size_ > 2 && mean_filter_size_ % 2 == 1)
  {
    cv::medianBlur(backprojection, backprojection, mean_filter_size_);
  }

  // perform thresholding
  cv::Mat binary;
  cv::threshold(backprojection, binary, binary_threshold_, 255, CV_THRESH_BINARY);

  // morphological operations
  cv::Mat binary_morphed = binary.clone();
  if (opening_element_size_ > 0)
  {
    cv::Mat element = cv::Mat::zeros(opening_element_size_, opening_element_size_, CV_8UC1);
    cv::circle(element, cv::Point(opening_element_size_ / 2, opening_element_size_ / 2), opening_element_size_ / 2, cv::Scalar(255), -1);
    cv::morphologyEx(binary_morphed, binary_morphed, cv::MORPH_OPEN, element);
  }
  if (closing_element_size_ > 0)
  {
    cv::Mat element = cv::Mat::zeros(closing_element_size_, closing_element_size_, CV_8UC1);
    cv::circle(element, cv::Point(closing_element_size_ / 2, closing_element_size_ / 2), closing_element_size_ / 2, cv::Scalar(255), -1);
    cv::morphologyEx(binary_morphed, binary_morphed, cv::MORPH_CLOSE, element);
  }  

  // create mask for invalid values
  std::vector<cv::Mat> hsv_channels;
  cv::split(hsv_image, hsv_channels);
  cv::Mat value = hsv_channels[2];
  cv::Mat min_value_mask;
  cv::threshold(value, min_value_mask, min_value_threshold_, 255, CV_THRESH_BINARY);

  // mask out low values in binary image
  cv::bitwise_and(min_value_mask, binary_morphed, binary_morphed);

  // Detect blobs in image
  std::vector< std::vector<cv::Point> > contours;
  cv::Mat contour_output = binary_morphed.clone();
  cv::findContours(contour_output, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

  ROS_INFO_STREAM("CONTOURS " << contours.size());

  cv::RotatedRect el;
  std::vector<cv::Point> biggest_blob;

  if (contours.size() > 0)
  {

    // 2 or more blobs detected. Delete too big and too small blobs
    /*std::vector< std::vector<cv::Point> >::iterator iter = contours.begin();
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
    }*/

    // Check that we keep having at least 1 contours
    if (contours.size() < 1)
    {
      ROS_DEBUG("[Tracker:] Blob filtering has removed too many blobs!");
    }
    else
    {
      // Sort the result by size
      std::sort(contours.begin(), contours.end(), color_circle_tracker::Utils::sort_vectors_by_size);

      // Get the biggest blob
      biggest_blob = contours[0];

      // Fit an ellipse
      el = cv::fitEllipse(biggest_blob);

      /*cv::Mat img = contour_image;
      cv::drawContours(img, biggest_blob, -1, color, 2);
      cv::ellipse( img, el, color, 2, 8 );
      cv::namedWindow("kk");
      cv::imshow("kk",img);
      cv::waitKey(3);*/
    }
  }
  else
  {
    ROS_DEBUG_STREAM("[Tracker:] Not enough points detected: " << contours.size() << " (3 needed).");
  }

  // debug purposes
  if (debug && !contour_image.empty())
  {
    /*for (size_t idx=0; idx<contours.size(); idx++)
    {
      cv::drawContours(contour_image, contours, idx, color, 2);
      cv::circle(contour_image, points[idx], 15, color, 2);
    }*/

    cv::drawContours(contour_image, contours, -1, color, 2);
    if(contours.size()>0)
      cv::ellipse(contour_image, el, cv::Scalar(255,0,0), 2, 8 );

    // Show images. First, convert to color
    cv::Mat binary_color, binary_morphed_color;
    cv::cvtColor(binary, binary_color, CV_GRAY2RGB);
    cv::cvtColor(binary_morphed, binary_morphed_color, CV_GRAY2RGB);

    // Concatenate horizontaly the images
    cv::Mat display_image(contour_image.size(), CV_8UC3);
    cv::hconcat(contour_image,binary_color,display_image);
    cv::hconcat(display_image,binary_morphed_color,display_image);

    // Create the window and the trackbars
    cv::namedWindow(tuning_gui_name_, 0);
    cv::setWindowProperty(tuning_gui_name_, CV_WND_PROP_ASPECTRATIO, CV_WINDOW_KEEPRATIO);
    cv::createTrackbar("mean_filter_size", tuning_gui_name_,  &mean_filter_size_, 255);
    cv::createTrackbar("binary_threshold", tuning_gui_name_,  &binary_threshold_, 255);
    cv::createTrackbar("closing_element_size", tuning_gui_name_,  &closing_element_size_, 255);
    cv::createTrackbar("opening_element_size", tuning_gui_name_,  &opening_element_size_, 255);
    cv::createTrackbar("min_value_threshold", tuning_gui_name_,  &min_value_threshold_, 255);
    cv::createTrackbar("min_blob_size", tuning_gui_name_,  &min_blob_size_, 255);
    cv::createTrackbar("max_blob_size", tuning_gui_name_,  &max_blob_size_, 255);

    // Get some reference values to position text in the image
    cv::Size sz = contour_image.size();
    int W = sz.width; // image width
    //int H = sz.height; // image height
    int scale = 2; // text scale
    int thickness = 4; // text font thickness
    int color = 255; // text colour (blue)
    int W0 = 10; // initial width 
    int H0 = 50; // initital height
    cv::putText(display_image, "Contours", cv::Point(W0,H0), cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness);
    cv::putText(display_image, "Binarized", cv::Point(W+W0,H0), cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness);
    cv::putText(display_image, "Morph", cv::Point(2*W+W0,H0), cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness);
    cv::imshow(tuning_gui_name_, display_image);
    cv::waitKey(5);
  }
  
  return el;
}

bool color_circle_tracker::Tracker::detectSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  camera_to_color_circle_.setIdentity();
  do_detection_ = false;
  toggle_detection_ = true;
  ROS_INFO("Service Detect requested.");
  return true;
}

bool color_circle_tracker::Tracker::startDetectionSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  camera_to_color_circle_.setIdentity();
  do_detection_ = true;
  ROS_INFO("Service Start Detection requested.");
  return true;
}

bool color_circle_tracker::Tracker::stopDetectionSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  do_detection_ = false;
  ROS_INFO("Service Stop Detection requested.");
  return true;
}