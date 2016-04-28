
#include "visual_odometry.hpp"

#ifndef   KEY_FRAME
#define   KEY_FRAME
#endif

void VisualOdometry::initialize(const cv::Mat& rgb0, const cv::Mat& depth0)
{
  //设置里程计的初始位姿
  frame_count = 0;
  key_frame_count = 0;
  trans.poseInitialize( Eigen::Isometry3d::Identity() );
  _ref_frame->prepareFrame( rgb0, depth0 );
  _prev_frame->prepareFrame( rgb0, depth0 );
  _cur_frame->prepareFrame( rgb0, depth0 );

}

void VisualOdometry::processFrame(const cv::Mat& rgb, const cv::Mat& depth)
{

#ifndef KEY_FRAME
//---------------------相邻帧连续计算----------------------
	frame_count++;
	std::cout << std::endl << "-------------------------------------"
						<< "--------------------------------------------------"
						<< "Frame index: " << frame_count << std::endl;

	Eigen::Isometry3d trans_cur_to_ref = 
					trans.ref_pose.inverse() * trans.cur_pose;
	trans.initial_motion_estimate = trans_cur_to_ref;

	std::swap( _cur_frame, _ref_frame );
	trans.ref_pose = trans.cur_pose;

	_cur_frame->prepareFrame( rgb, depth );

	_estimator->estimateMotion( _ref_frame,
															_cur_frame,
           trans.initial_motion_estimate );

	Eigen::Isometry3d to_reference = _estimator->getMotionEstimate();
	trans.cur_pose = trans.ref_pose * to_reference;
//---------------------相邻帧连续计算----------------------




#else

//---------------------关键帧技术----------------------
	frame_count++;
	key_frame_count++;
	std::cout << std::endl << "-------------------------------------"
						<< "--------------------------------------------------"
						<< "Frame index: " << frame_count << std::endl;

//--------------速度一致性模型------------------
	Eigen::Isometry3d trans_cur_to_prev =
					trans.prev_pose.inverse() * trans.cur_pose;

  //变换到当前帧
  if( FAILED != _estimator->getMotionEstimateStatus() )
  {
		trans.prev_pose = trans.cur_pose;
		std::swap( _prev_frame, _cur_frame );
	}
	
	trans.initial_motion_estimate = 
										trans.ref_pose.inverse() * 
										trans.prev_pose * 
										trans_cur_to_prev;


	_cur_frame->prepareFrame( rgb, depth );

	_estimator->estimateMotion( _ref_frame,
															_cur_frame,
           trans.initial_motion_estimate );


  if( _estimator->isMotionEstimateValid() )
  {
		Eigen::Isometry3d to_reference = _estimator->getMotionEstimate();
		trans.cur_pose = trans.ref_pose * to_reference;
  } 
  else if( MOTION_TOO_LARGE == _estimator->getMotionEstimateStatus() ) 
  {
	  std::cout << "_________________________"
	  					<< "motion too large, change key frame"
	  					<< "_________________________"
	  					<< std::endl;

    vec_key.push_back( key_frame_count );
    key_frame_count = 0;

    //直接切换关键帧
		// Eigen::Isometry3d to_reference = _estimator->getMotionEstimate();
		// trans.cur_pose = trans.ref_pose * to_reference;

		// std::swap( _prev_frame, _ref_frame );
  	// trans.ref_pose = trans.prev_pose;

		std::swap( _prev_frame, _ref_frame );
    trans.ref_pose = trans.prev_pose;
		trans.initial_motion_estimate = trans_cur_to_prev;

		_estimator->estimateMotion( _ref_frame,
																_cur_frame,
           trans.initial_motion_estimate );

    if( _estimator->isMotionEstimateValid() ) 
	  {
			Eigen::Isometry3d to_reference = _estimator->getMotionEstimate();
			trans.cur_pose = trans.ref_pose * to_reference;
	  }
	  else
	  {
	  	std::cout << "This frame compute failed,`motion too large``continue```" << std::endl;
	  	_estimator->estimate_status = FAILED;
	  }

  }
  else if( INSUFFICIENT_INLIERS == _estimator->getMotionEstimateStatus() )
  {
    //变换参考帧
	  std::cout << ".........................."
							<< "inliers too fewer, change key frame"
							<< "..........................."
							<< std::endl;

    vec_key.push_back( key_frame_count );
    key_frame_count = 0;

		std::swap( _prev_frame, _ref_frame );
    trans.ref_pose = trans.prev_pose;
		trans.initial_motion_estimate = trans_cur_to_prev;

		_estimator->estimateMotion( _ref_frame,
																_cur_frame,
           trans.initial_motion_estimate );

    if( _estimator->isMotionEstimateValid() ) 
	  {
			Eigen::Isometry3d to_reference = _estimator->getMotionEstimate();
			trans.cur_pose = trans.ref_pose * to_reference;
	  }
	  else
	  {
	  	std::cout << "This frame compute failed,```continue```" << std::endl;
	  	_estimator->estimate_status = FAILED;
	  }
  }
//---------------------关键帧技术----------------------
#endif


}


bool VisualOdometry::save(const cv::String& path)
{


  char s[10];
  sprintf( s, "%d", frame_count );
  // std::string sss(ccc);
  
  std::string rgb_filename    = path + "/rgb/rgb-"     + s + ".png";
  std::string depth_filename  = path + "/depth/depth-" + s + ".png";
  // std::string header_filename = path + "/header.yml";
  // std::string intr_filename   = path + "/intr.yml"; 
  std::string cloud_filename  = path + "/cloud/cloud-" + s+ ".pcd";
  std::string depth_uncertainty_filename  = path + "/depth_uncertainty/uncertainty-" + s+ ".png";


  std::string tri_filename        = path + "/triangulation/tri-" + s +".png";
  std::string inlier_filename     = path + "/inlier-match/inlier-" + s +".png";
  std::string tri_inlier_filename = path + "/tri-inlier-match/tri-inlier-" + s +".png";



	// cv::imwrite( inlier_filename, _estimator->inlier_match_picture );

	// cv::imwrite( tri_filename, _estimator->_matcher->tri );
	// cv::imwrite( tri_inlier_filename, _estimator->inlier_match_picture );







  // // save header
  // cv::FileStorage fs_h(header_filename, cv::FileStorage::WRITE);
  // fs_h << "frame_id"   << frame.header.frame_id;
  // fs_h << "seq"        << (int)frame.header.seq;
  // fs_h << "stamp_sec"  << (int)frame.header.stamp.sec;
  // fs_h << "stamp_nsec" << (int)frame.header.stamp.nsec;
  // fs_h << "index"      << frame.index;

  // save images 
  // cv::imwrite( rgb_filename,   _cur_frame->rgb_img);
  // cv::imwrite( depth_filename, _cur_frame->depth_img);
  // cv::imwrite( depth_uncertainty_filename, _cur_frame->depth_uncertainty);

  // pcl::io::savePCDFile( cloud_filename.c_str(), _cur_frame->cloud_one_frame );
  // // save intrinsic matrix
  // cv::FileStorage fs_mat(intr_filename, cv::FileStorage::WRITE);
  // fs_mat << "intr" << frame.intr;

  return true;
}


// void VisualOdometry::drawImage()

