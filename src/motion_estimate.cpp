
#include "motion_estimate.hpp"

#define   KEY_FRAME
#define   USE_RANSAC
#define   USE_HORN_ABSOLUTE_ORIENTATION

static Eigen::Isometry3d rt2eigen( cv::Mat& rvec, cv::Mat& tvec )
{
  cv::Mat R;
  cv::Rodrigues( rvec, R );  // 3*1
  Eigen::Matrix3d r;
  cv::cv2eigen(R, r);

	Eigen::AngleAxisd angle( r );
  Eigen::Isometry3d T( angle );

  T(0,3) = tvec.at<double>(0,0);   // 3*1
  T(1,3) = tvec.at<double>(1,0); 
  T(2,3) = tvec.at<double>(2,0);
  return T;
}

static void eigen2rt( const Eigen::Isometry3d& tp, 
                    cv::Mat& rvec, cv::Mat& tvec )
{
  cv::Mat R;
  Eigen::Matrix3d r( tp.rotation() );
  cv::eigen2cv( r, R );
  cv::Rodrigues( R, rvec );  // 3*1

  Eigen::Vector3d tt( tp.translation() );
  cv::eigen2cv( tt, tvec );  // 3*1
}


bool MotionEstimator::is_motion_too_large( Eigen::Isometry3d& t )
{
  Eigen::Vector3d tt = t.translation();
  Eigen::AngleAxisd rr(t.rotation());
  
  if(  fabs(tt.norm()) > motion_translation  && 
      fabs(rr.angle() * 180.0 / M_PI) > motion_angle )
  {
    trans_count++; angle_count++; return true;
  }
  else if(  fabs(tt.norm() ) > motion_translation )
  {
    trans_count++; return true;
  }
  else if ( fabs(rr.angle() * 180.0 / M_PI) > motion_angle )
    { angle_count++; return true;}
  else
    return false;

  // if(  fabs(tt.norm()) > motion_translation
  //   || fabs(rr.angle() * 180.0 / M_PI) > motion_angle )
  //   return true;
  // else
  //   return false;
}


void MotionEstimator::estimateMotion( Frame* reference_frame,
                                      Frame* target_frame,
                    const Eigen::Isometry3d& init_motion_est )
                    // const Eigen::MatrixXd init_motion_cov)
{

	_matcher->match(  reference_frame,
                    target_frame,
                    init_motion_est );

//----判断条件，是否为关键帧
#ifdef KEY_FRAME
  if( _matcher->num_Matches < num_inlier_threshold )
  {
    estimate_status = INSUFFICIENT_INLIERS;
    inliers_count++;
    return;
  }
#endif
/*
//----------------------3D-3D运动估计--------------------
  timer("estimate_3Dto3D");
  Eigen::Isometry3d motion_estimate_3Dto3D;

  Eigen::MatrixXd tar_xyz(3, _matcher->matches.size());
  Eigen::MatrixXd ref_xyz(3, _matcher->matches.size());
  for(int i =0; i<_matcher->matches.size(); i++)
  {
    ref_xyz.col(i) = reference_frame->kp_means[ _matcher->matches[i].queryIdx ];
    tar_xyz.col(i) =    target_frame->kp_means[ _matcher->matches[i].trainIdx ];
  }

#ifdef USE_HORN_ABSOLUTE_ORIENTATION
  //速度快
  absolute_orientation_horn( ref_xyz, tar_xyz, motion_estimate_3Dto3D );
#else
  Eigen::Matrix4d ume_estimate = Eigen::umeyama( ref_xyz, tar_xyz, false );
  motion_estimate_3Dto3D = Eigen::Isometry3d(ume_estimate);
#endif

  // std::cout << motion_estimate_umeyamad.matrix() << std::endl;
  timer("estimate_3Dto3D");
  num_inliers = _matcher->matches.size();
  motion_estimate = motion_estimate_3Dto3D;

//----------------------3D-3D运动估计--------------------
*/


//----------重投影最小误差优化--------------------
	std::vector<cv::Point3f> pts_obj, new_pts_obj;
	std::vector<cv::Point2f> pts_img,project_pts, new_pts_img;
  cv::Mat rvec, tvec, inliers;
  // eigen2rt( motion_estimate_umeyamad, rvec, tvec);

#ifndef USE_RANSAC
  timer("PnP");
  for( int i=0; i < _matcher->num_Matches; i++ )
  {
    cv::Point3f p3d =    target_frame->point_xyz[ _matcher->matches[i].trainIdx ];
    cv::Point2f p2d = reference_frame->keypoints[ _matcher->matches[i].queryIdx ].pt;
    pts_obj.push_back(p3d);
    pts_img.push_back(p2d);
  }
  cv::solvePnP( pts_obj, 
                pts_img, 
                cameraMatrix, 
                cv::Mat(), 
                rvec, tvec,  // 3*1
                false  );

//-----位姿refine-------------
//-----利用这个估计，去除不符合匹配的点，再重新做一次计算
  cv::projectPoints(  pts_obj,
                      rvec, tvec,
                      cameraMatrix, cv::Mat(),
                      project_pts  );
  std::vector<bool> boolInlier;
  boolInlier.resize( _matcher->num_Matches, false);

  for( int i=0; i < _matcher->num_Matches; i++ )
  {
    cv::Point2f dis( pts_img[i].x - project_pts[i].x, pts_img[i].y - project_pts[i].y);
    double dis2 = dis.x * dis.x + dis.y * dis.y;
    if( dis2 < 2.25 )
    {  
      boolInlier[i] = true;
      new_pts_obj.push_back(pts_obj[i]);
      new_pts_img.push_back(pts_img[i]);
    }
  }

  num_inliers = new_pts_img.size();
  if( num_inliers < num_inlier_threshold )
  {
    estimate_status = INSUFFICIENT_INLIERS;
    return;
  }

  cv::solvePnP( new_pts_obj, 
                new_pts_img, 
                cameraMatrix, 
                cv::Mat(), 
                rvec, tvec,  // 3*1
                false  );
//--------以上利用这个估计，去除不符合匹配的点，再重新做一次计算

  std::cout << "inliers: "<< num_inliers << std::endl;
  motion_estimate = rt2eigen( rvec, tvec );
  timer("PnP");

#else
  //----------------------------------------
  //-----RANSAC算法--------
  timer("RANSAC_PnP");
  for( size_t i=0; i < _matcher->num_Matches; i++ )
  {
    cv::Point3f p3d =    target_frame->point_xyz[ _matcher->matches[i].trainIdx ];
    cv::Point2f p2d = reference_frame->keypoints[ _matcher->matches[i].queryIdx ].pt;
    pts_obj.push_back(p3d);
    pts_img.push_back(p2d);
  }

  // 求解pnp
  cv::solvePnPRansac( pts_obj, 
  										pts_img, 
  										cameraMatrix, 
  										cv::Mat(), 
  										rvec, tvec,
  										false,
  									  500,   //迭代次数
  										1,   //重投影误差阈值
  										0.99, //置信度
  										inliers );
  num_inliers = inliers.rows;
  std::cout << "inliers: "<< num_inliers << std::endl;
  motion_estimate = rt2eigen( rvec, tvec );
  timer("RANSAC_PnP");


//-----下面是调试ba的程序------------
  // Eigen::AngleAxisd ro( motion_estimate.rotation() );
  // std::cout << "   translation: " << motion_estimate.translation().norm()
  //           << "   rotation: " << ro.angle() * 180.0 / M_PI <<std::endl;

  // inlier_match.clear();
  // // inlier_match.reserve( pts_obj.size() );
  // for( int i=0; i < num_inliers; i++ )
  //   inlier_match.push_back( _matcher->matches[ inliers.at<int>(i,0) ] );

  // Eigen::Isometry3d  tt( motion_estimate );
  //   timer("ba_Optimal");
  // _ba_g2o->ba_Optimal(  reference_frame,
  //                       target_frame,
  //                       inlier_match,
  //                       tt );
  //   timer("ba_Optimal");
  // Eigen::AngleAxisd rr( tt.rotation() );
  // std::cout << "   translation: " << tt.translation().norm()
  //             << "   rotation: " << rr.angle() * 180.0 / M_PI <<std::endl;


#endif

//----判断条件，是否为关键帧
#ifdef KEY_FRAME
  if( num_inliers < num_inlier_threshold
      /* ||  double(num_inliers) / _matcher->num_Matches < 0.4*/ )
  {
    estimate_status = INSUFFICIENT_INLIERS;
    inliers_count++;
    return;
  }
  else if( is_motion_too_large( motion_estimate ) )
  {
    Eigen::AngleAxisd r( motion_estimate.rotation() );
    std::cout << "   translation: " << motion_estimate.translation().norm()
              << "   rotation: " << r.angle() * 180.0 / M_PI <<std::endl;
    estimate_status = MOTION_TOO_LARGE;
  }
  else
    estimate_status = SUCCESS;
#else
// std::cout << "输出5" << std::endl;
    estimate_status = SUCCESS;
#endif
//------------------------------------------------------
//------------------画出内点匹配图-----------------------
  // inlier_match.clear();
  // inlier_match.reserve( pts_obj.size() );
  // for( int i=0; i < num_inliers; i++ )
  //   inlier_match.push_back( _matcher->matches[ inliers.at<int>(i,0) ] );
  // cv::drawMatches( reference_frame->rgb_img, 
  //                  reference_frame->keypoints, 
  //                  target_frame->rgb_img, 
  //                  target_frame->keypoints, 
  //                  inlier_match, 
  //                  inlier_match_picture,
  //                  cv::Scalar(0,0,255),
  //                  cv::Scalar(255,0,0) );
//------------------------------------------------------

}


void MotionEstimator::drawInlierMatch( 
                          Frame* reference_frame,
                          Frame* target_frame)
{

}

