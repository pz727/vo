
#include "feature_matcher.hpp"

#define   USE_MAX_CONNECTED_DOMAIN
// #define   USE_MAX_CLIQUE

void FeatureMatcher::match( Frame* reference_frame,
            								Frame*    target_frame,
          const Eigen::Isometry3d& init_motion_est )
{
  init_matches.clear();
  no_repeat_matches.clear();
  matches.clear();
  final_matches.clear();

  bf_matcher.match( reference_frame->descriptors, target_frame->descriptors, init_matches );
  std::cout<<"Find init_match "<< init_matches.size() << " matches."<< std::endl;
  num_InitMatches = init_matches.size();

  matchFilter(  reference_frame,
                target_frame,
                init_motion_est   );
  num_NoRepeatMatches = no_repeat_matches.size();

#ifdef USE_MAX_CONNECTED_DOMAIN
  timer("max_connected");
  _maxConDom->computeMaxConnectedDomain(
            reference_frame,
            target_frame,
            no_repeat_matches    );

  num_Matches = _maxConDom->nums_triangle;

  std::cout << "final matches number: "<< num_Matches << std::endl;
  for( int i=0; i < num_Matches; i++ )
  {
    matches.push_back( no_repeat_matches[ _maxConDom->final_domain[i] ] );
  }
  timer("max_connected");

#else
  num_Matches = num_NoRepeatMatches;
  matches = no_repeat_matches;

#endif


#ifdef USE_MAX_CLIQUE
  timer("max_clique");
  computeMaximallyConsistentClique( 
                  reference_frame,
                  target_frame   );


  timer("max_clique");


#else
  num_final_Matches = num_Matches;
  final_matches = matches;

#endif



//----------------------------------------------------
//-------------画出，最后的匹配图---------------------
  // cv::drawMatches( reference_frame->rgb_img, 
  //                  reference_frame->keypoints, 
  //                  target_frame->rgb_img, 
  //                  target_frame->keypoints, 
  //                  matches, 
  //                  imgMatch,
  //                  cv::Scalar(0,0,255),
  //                  cv::Scalar(255,0,0) );

  // cv::imshow( "imgMatch", imgMatch );
  // cv::waitKey(0);  
//-------------------------------------------------------

}






static bool matchCompare(const  cv::DMatch& m1, const cv::DMatch& m2 )
{
  return m1.distance < m2.distance;
}

cv::Point2f FeatureMatcher::point3Dto2D( 
          cv::Point3f& point_xyz, 
    const Eigen::Isometry3d & trans    )
{
  cv::Point2f p; // 3D 点
  Eigen::Vector4d p_xyz1( point_xyz.x, point_xyz.y, point_xyz.z, 1 );
  //变换到参考坐标系下
  Eigen::Matrix<double, 3, 4> K = pCamera->toProjectionMatrix();
  Eigen::Vector3d p_uvw =  K * trans.matrix() * p_xyz1;

  p.x = p_uvw(0) / p_uvw(2);
  p.y = p_uvw(1) / p_uvw(2);
  return p;
}

void FeatureMatcher::matchFilter(
          Frame* reference_frame,
          Frame* target_frame,
    const Eigen::Isometry3d& init_motion_est   )
{
  std::sort( init_matches.begin(),init_matches.end(), matchCompare );

  for( int i=0; i < init_matches.size(); i++ )
  {
    //匹配距离太远，则全部丢弃
    if( init_matches[i].distance > 10 * init_matches[0].distance )
      break;

    // if( no_repeat_matches.size() >=200)
    //   break;

    cv::Point2f ref_p1 = reference_frame->keypoints[ init_matches[i].queryIdx ].pt;
    //限制最大特征运动
    // cv::Point3f tar_p3d = target_frame->point_xyz[ init_matches[i].trainIdx ];
    // cv::Point2f tar_p1 = point3Dto2D( tar_p3d, init_motion_est );

    // cv::Point2f diff_p1 = ref_p1 - tar_p1;
    // double dis_sq = diff_p1.x * diff_p1.x + diff_p1.y * diff_p1.y;
    // if( dis_sq > max_feature_motion * max_feature_motion )
    //   {     continue;   }

    //自己的局部最小抑制
    bool flag( false );
    for( int j=0; j < i; j++ )
    {
      cv::Point2f ref_p2 = reference_frame->keypoints[ init_matches[j].queryIdx ].pt;
      cv::Point2f distance_p = ref_p1 - ref_p2;
      double distance = distance_p.x * distance_p.x + distance_p.y * distance_p.y;
      if( distance < max_feature_distance * max_feature_distance )
        {   flag = true;    break;  }
    }
    if( flag == true )
      continue;

    no_repeat_matches.push_back( init_matches[i] );
  }
  
  std::cout<<"no_repeat_matches "<< no_repeat_matches.size() <<" matches."<<std::endl;
}







// used for sorting feature matches.
static bool consistencyCompare(const Max_clique_match &ca, const Max_clique_match& cb)
{
  return ca.compatibility_degree > cb.compatibility_degree;
}

void FeatureMatcher::computeMaximallyConsistentClique(
						Frame* reference_frame,
            Frame* target_frame )
{
	//清空，初始化Max_clique_match
	max_clique.clear();
  for (int m_ind = 0; m_ind < num_Matches; m_ind++)
  {
  	Max_clique_match max_clique_tmp;
  	max_clique_tmp.id = m_ind;
  	max_clique_tmp.clique_match = matches[m_ind];
  	max_clique_tmp.consistency_vec.resize(num_Matches,0);
  	max_clique_tmp.in_clique = false;
  	max_clique_tmp.compatibility_degree = 0;

  	max_clique.push_back( max_clique_tmp );
  }

  // FIXME using both homogeneous and cartesian coordinates is a bit gross here...
  for (int m_ind = 0; m_ind < max_clique.size(); m_ind++)
  {
    Max_clique_match& match = max_clique[m_ind];

    cv::Point3f cv_ref_xyz = 
    		reference_frame->point_xyz[match.clique_match.queryIdx];
    cv::Point3f cv_tar_xyz = 
    		target_frame->point_xyz[match.clique_match.trainIdx];

    for (int m_ind2 = m_ind + 1; m_ind2 < max_clique.size(); m_ind2++) 
    {
      Max_clique_match& match2 = max_clique[m_ind2];

	    cv::Point3f cv_ref_xyz2 = 
	    		reference_frame->point_xyz[match2.clique_match.queryIdx];
	    cv::Point3f cv_tar_xyz2 = 
	    		target_frame->point_xyz[match2.clique_match.trainIdx];

      cv::Point3f cv_ref_dist = cv_ref_xyz - cv_ref_xyz2;
      cv::Point3f cv_tar_dist = cv_tar_xyz - cv_tar_xyz2;
      Eigen::Vector3f eref_dist(cv_ref_dist.x, cv_ref_dist.y, cv_ref_dist.z);
      Eigen::Vector3f etar_dist(cv_tar_dist.x, cv_tar_dist.y, cv_tar_dist.z);

      double ref_dist = eref_dist.norm();
      double tar_dist = etar_dist.norm();

      if ( fabs(ref_dist - tar_dist) < clique_inlier_threshold ) 
      {
        match.consistency_vec[match2.id] = 1;
        match.compatibility_degree++;
        match2.consistency_vec[match.id] = 1;
        match2.compatibility_degree++;
      }
    }
  }

  // sort the features based on their consistency with other features
  std::sort(max_clique.begin(), max_clique.end(), consistencyCompare);

  // pick the best feature and mark it as an inlier
  Max_clique_match &best_candidate = max_clique[0];
  best_candidate.in_clique = true;
  num_final_Matches = 1;

  // start a list of quick-reject features (features that are known to be
  // inconsistent with any of the existing inliers)
  int reject[max_clique.size()];
  std::fill(reject, reject+max_clique.size(), 0);
  for (int m_ind = 1; m_ind < max_clique.size(); m_ind++) 
  {
    int other_id = max_clique[m_ind].id;
    if ( !best_candidate.consistency_vec[other_id] )
      reject[other_id] = 1;
  }

  // now start adding inliers that are consistent with all existing
  // inliers
  for ( int m_ind = 1; m_ind < max_clique.size(); m_ind++ )
  {
		Max_clique_match& cand = max_clique[m_ind];
    // if this candidate is consistent with fewer than the existing number
    // of inliers, then immediately stop iterating since no more features can
    // be inliers
    if ( cand.compatibility_degree < num_final_Matches )
      break;

    // skip if it's a quick reject
    if ( reject[cand.id] )
      continue;

    cand.in_clique = true;
    num_final_Matches++;

    // mark some more features for rejection
    for (int r_ind = m_ind + 1; r_ind < max_clique.size(); r_ind++) 
    {
      int other_id = max_clique[r_ind].id;
      if ( !reject[other_id] && !cand.consistency_vec[other_id] )
        reject[other_id] = 1;
    }
  }

  for( int i=0; i < max_clique.size(); i++ )
  {
  	if( max_clique[i].in_clique )
  		final_matches.push_back(max_clique[i].clique_match);
  }

  std::cout<<"final final_matches: "<< final_matches.size() <<" matches."<<std::endl;

}









