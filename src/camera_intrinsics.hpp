#ifndef __camera_intrinsics_hpp__
#define __camera_intrinsics_hpp__

#include <Eigen/Geometry>

/**
 * \ingroup FovisCore
 * \brief Intrinsic parameters for a pinhole camera with plumb-bob distortion model.
 *
 */
struct CameraIntrinsicsParameters
{
  CameraIntrinsicsParameters() :
    width(0), height(0), fx(0), fy(0), cx(0), cy(0), factor(1),
    k1(0), k2(0), k3(0), p1(0), p2(0)
  {}

  /**
   * \code
   * [ fx 0  cx 0 ]
   * [ 0  fy cy 0 ]
   * [ 0  0  1  0 ]
   * \endcode
   *
   * \return a 3x4 projection matrix that transforms 3D homogeneous points in
   * the camera frame to 2D homogeneous points in the image plane.
   */
  Eigen::Matrix<double, 3, 4> toProjectionMatrix() const
  {
    Eigen::Matrix<double, 3, 4> result;
    result <<
      fx,  0, cx, 0,
       0, fy, cy, 0,
       0,  0,  1, 0;
    return result;
  }

  Eigen::Matrix3d toCVCameraMatrix() const
  {
    Eigen::Matrix3d result;
    result <<
      fx,  0, cx,
       0, fy, cy,
       0,  0,  1;
    return result;
  }

  int width,height;

  double fx,fy;
  double cx,cy;

  double factor;

  double k1,k2,k3;

  double p1,p2;

};

#endif
