#ifndef __motion_estimation_hpp__
#define __motion_estimation_hpp__

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h> //用于fabs函数

#include "opencv2/core/eigen.hpp"
// #include "opencv2/calib3d/calib3d.hpp"

#include "frame.hpp"
#include "camera_intrinsics.hpp"
#include "feature_matcher.hpp"
#include "absolute_orientation_horn.hpp"
// #include "ba_g2o.hpp"

enum MotionEstimateStatusCode
{
  NO_DATA,
  SUCCESS,
  FAILED,
  MOTION_TOO_LARGE,
  INSUFFICIENT_INLIERS,
  OPTIMIZATION_FAILURE,
  REPROJECTION_ERROR
};


class MotionEstimator
{
  public:
    MotionEstimator(CameraIntrinsicsParameters* input_camera)
    : trans_count (0), angle_count(0), inliers_count(0)
    {
      pCamera  = input_camera;
      _matcher = new FeatureMatcher( pCamera );
      // _ba_g2o  = new BA_G2O( pCamera );
      estimate_status = NO_DATA;

      Eigen::Matrix3d cameraEigen = pCamera->toCVCameraMatrix();
      cv::eigen2cv( cameraEigen, cameraMatrix );

      motion_translation = getDoubleFromYML("../parameter.yml","motion_translation");
      motion_angle = getDoubleFromYML("../parameter.yml","motion_angle");
      num_inlier_threshold = getIntFromYML("../parameter.yml","num_inlier_threshold");
    }

    ~MotionEstimator()
    {
      delete _matcher;
      // delete _ba_g2o;
    }

    void estimateMotion(Frame* reference_frame,
                        Frame* target_frame,
                        const Eigen::Isometry3d& init_motion_est );
                        // const Eigen::MatrixXd init_motion_cov);


    bool is_motion_too_large( Eigen::Isometry3d& t );

    bool isMotionEstimateValid() const 
    {
      return estimate_status == SUCCESS;
    }

    MotionEstimateStatusCode getMotionEstimateStatus() const 
    {
      return estimate_status;
    }

    Eigen::Isometry3d getMotionEstimate() const
    {
      return motion_estimate;
    }

    const Eigen::MatrixXd& getMotionEstimateCov() const 
    {
      return motion_estimate_covariance;
    }

    int getNumInliers() const 
    {
      return num_inliers;
    }

    int getNumInitMatches() const 
    {
      return _matcher->getNumInitMatches();
    }

    int getNumNoRepeatMatches() const 
    {
      return _matcher->getNumNoRepeatMatches();
    }

    int getNumMatches() const 
    {
      return _matcher->getNumMatches();
    }



    // int getNumReprojectionFailures() const {
    //   return _num_reprojection_failures;
    // }

    // double getMeanInlierReprojectionError() const {
    //   return _mean_reprojection_error;
    // }


  // private:
    // void matchFeatures(PyramidLevel* ref_level, PyramidLevel* target_level);
    // void computeMaximallyConsistentClique();
    // void estimateRigidBodyTransform();
    // void refineMotionEstimate();
    // void computeReprojectionError();

    //内点匹配图
    std::vector<cv::DMatch>   inlier_match;
    cv::Mat                   inlier_match_picture;      
    void drawInlierMatch( Frame* reference_frame,
                          Frame* target_frame);

    // inlier count
    int num_inliers;

    FeatureMatcher* _matcher;
    // BA_G2O        * _ba_g2o;

    Eigen::Isometry3d motion_estimate;
    Eigen::MatrixXd   motion_estimate_covariance;

    MotionEstimateStatusCode estimate_status;

    CameraIntrinsicsParameters* pCamera;
    cv::Mat cameraMatrix;

    //  参数
    double  motion_translation;
    double  motion_angle;
    int     num_inlier_threshold;

    int     trans_count, angle_count, inliers_count;

};


#endif