#ifndef __visual_odometry_hpp__
#define __visual_odometry_hpp__

#include <Eigen/Geometry>

#include "camera_intrinsics.hpp"
#include "frame.hpp"
#include "motion_estimate.hpp"
#include "timer.hpp"


/**
 * Utility class so that the VisualOdometry class not need
 * EIGEN_MAKE_ALIGNED_OPERATOR_NEW.
 */
struct Transformation
{
  Eigen::Isometry3d cur_pose;

  Eigen::Isometry3d ref_pose;

  Eigen::Isometry3d prev_pose;

  // best estimate of motion from current to previous frame
  Eigen::Isometry3d motion_estimate;
  // the 6x6 estimate of the covriance [x-y-z, roll-pitch-yaw];
  Eigen::MatrixXd motion_estimate_covariance;

  Eigen::Isometry3d initial_motion_estimate;
  Eigen::MatrixXd   initial_motion_cov;

  void poseInitialize( const Eigen::Isometry3d& init_pose = 
                       Eigen::Isometry3d::Identity() )
  {
    cur_pose = init_pose;
    prev_pose = cur_pose;
    ref_pose = cur_pose;
  }
};


class VisualOdometry
{
  public:

    VisualOdometry(const CameraIntrinsicsParameters& input_camera)
    {
      camera = input_camera;

      _ref_frame  = new Frame( &camera );
      _prev_frame = new Frame( &camera );
      _cur_frame  = new Frame( &camera );
      _estimator  = new MotionEstimator( &camera );
    }

    ~VisualOdometry()
    {
      delete _estimator;
      delete _ref_frame;
      delete _prev_frame;
      delete _cur_frame;
    }

    /**
     * process an input image and estimate the 3D camera motion between \p gray
     * and the frame previously passed to this method.  The estimated motion
     * for the very first frame will always be the identity transform.
     *
     */
    void initialize( const cv::Mat& rgb0, const cv::Mat& depth0 );

    void processFrame( const cv::Mat& rgb, const cv::Mat& depth );

    bool save( const cv::String& path );
    /**
     * Retrieves the integrated pose estimate.  On initialization, the camera
     * is positioned at the origin, with +Z pointing along the camera look
     * vector, +X to the right, and +Y down.
     */
    const Eigen::Isometry3d& getPose()
    {
      return trans.cur_pose;
    }

    int getFrameCount()
    {
      return frame_count;
    }


    MotionEstimateStatusCode getStatus() const 
    {
      return _estimator->getMotionEstimateStatus();
    }


    const Eigen::Isometry3d & getInitialEstimation() const 
    {
      return trans.initial_motion_estimate;
    }

    int getNumOfInliers() const 
    {
      return _estimator->getNumInliers();
    }


    int getNumInitMatches() const 
    {
      return _estimator->getNumInitMatches();
    }

    int getNumNoRepeatMatches() const 
    {
      return _estimator->getNumNoRepeatMatches();
    }

    int getNumMatches() const 
    {
      return _estimator->getNumMatches();
    }

   // private:

    Frame* _ref_frame;
    Frame* _prev_frame;
    Frame* _cur_frame;

    MotionEstimator* _estimator;

    Transformation trans;
    CameraIntrinsicsParameters camera;


    //统计参数
    int frame_count;
    int key_frame_count;
    std::vector<int>  vec_key;

};

#endif
