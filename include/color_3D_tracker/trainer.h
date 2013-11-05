/**
 * @file
 * @brief color_circle trainer presentation.
 */

#ifndef TRAINER_H
#define TRAINER_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/stereo_camera_model.h>

#include "color_3D_tracker/stereo_processor.h"
#include "color_3D_tracker/trainer.h"

using namespace std;
using namespace cv;

namespace color_3D_tracker
{

class Trainer : public StereoImageProcessor
{

public:

  // Trainer states
	enum Mode
	{
	  DISPLAY_VIDEO,
	  AWAITING_TRAINING_IMAGE,
	  SHOWING_TRAINING_IMAGE,
	  PAINTING,
	  ROI_SELECTED,
	  TRAINED
	};

	Mat image; //!> showing image

  // Constructors
  Trainer(const string transport);

  // Destructor
  ~Trainer();

private:

  // Trainer HSV values
  string trained_model_path_;
  int num_hue_bins_;
  int num_sat_bins_;
  int num_val_bins_;

  int training_status_;               //!> Defines the trainer Mode

  Mat training_image_;            //!> Image HSV used to train

  Point roi_rectangle_origin_;    
  Rect roi_rectangle_selection_;

  MatND model_histogram_;         //!> Histogram of the trained model

  void stereoImageCallback(
      const sensor_msgs::ImageConstPtr& l_image_msg,
      const sensor_msgs::ImageConstPtr& r_image_msg,
      const sensor_msgs::CameraInfoConstPtr& l_info_msg,
      const sensor_msgs::CameraInfoConstPtr& r_info_msg); //!> Image callback
  MatND train(const Mat& image);
  int detect(const MatND& hist, const Mat& image);
  static void staticMouseCallback(int event, int x, int y, int flags, void* param);
  void mouseCallback(int event, int x, int y, int flags, void* param);
  MatND calculateHistogram(const Mat& image, 
	                             const int bins[],  
	                             const Mat& mask);
  void showHSVHistogram(const MatND& histogram,
                        const string& name_hs, 
                        const string& name_hv);
};

} // namespace

#endif // TRACKER_H
