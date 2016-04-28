
#include "triangulation.hpp"

// #define  MATCH_SELECTED_IMAGE
// #define  ORIGINAL_MESH

// 面积计算
double MaxConnectedDomain::areaOfDomain( const std::vector<int> & ca )
{
  Gt gt;
  std::vector<Point> points_ca( ca.size() );
  Polygon_2 p_ca;
  for(size_t i =0; i < ca.size(); i++ )
  {
    points_ca.push_back( points[ ca[i] ].first );
  }

  CGAL::convex_hull_2( points_ca.begin(),
                       points_ca.end(), 
                       std::back_inserter(p_ca),
                       gt );
  // std::cout << "输出4" << std::endl;
  return p_ca.area();
}


// used for sorting feature matches.
static bool vecSizeCompare(const std::vector<int> &ca, const std::vector<int>& cb)
{
  return ca.size() > cb.size();
}

void MaxConnectedDomain::computeMaxConnectedDomain(
            Frame* reference_frame,
            Frame* target_frame,
            std::vector<cv::DMatch>& matches)
{
	int num_points = matches.size();
  points.clear();
  for( int i=0; i < num_points; i++ )
  {
    Eigen::Vector3d& p_ind = reference_frame->kp_means[ matches[i].queryIdx ];
    Point p_dt( p_ind(0), p_ind(1), p_ind(2) );
    points.push_back( std::make_pair(p_dt,i) );
  }

  Delaunay dt( points.begin(), points.end() );
  Delaunay::Finite_edges_iterator eit;

//-----------------------------------------------------------
//------------------画出原始的三角剖分图----------------------
#ifdef  ORIGINAL_MESH
	cv::drawKeypoints(  reference_frame->rgb_img,
                  		reference_frame->keypoints, 
                    	img_original_triangulation_mesh,
                    	cv::Scalar(0,255,0) );
  for ( eit = dt.finite_edges_begin(); 
        eit != dt.finite_edges_end(); 
        ++eit )
        //eit->first表示Face_handle
    //eit->second表示int
    //Vertex_handle   vertex (int i) const
    //边的第一个顶点索引
    //eit->first->vertex( eit->first->cw( eit->second) )->info();
    //边的第二个顶点索引
    //std::cerr << eit->first->vertex( eit->first->ccw(eit->second) )->info();
  { 
    cv::Point2f pt1 = 
      reference_frame->keypoints[ matches[ eit->first->vertex( eit->first->cw( eit->second) )->info()].queryIdx ].pt;
    cv::Point2f pt2 = 
      reference_frame->keypoints[ matches[ eit->first->vertex( eit->first->ccw( eit->second))->info()].queryIdx ].pt;

    cv::line( img_original_triangulation_mesh, pt1, pt2, cv::Scalar(0,0,255));
  }

  cv::imshow( "img_original_triangulation_mesh", img_original_triangulation_mesh);
  cv::waitKey(0);

#endif
//-----------------------画出原始的三角剖分图-------------------
//-------------------------------------------------------------

//------------建立图论模型------------------
//-----------求取最大连通域------------------
  Graph G;

  //开始实际的边剔除
  points_inlier.clear();
  for ( eit = dt.finite_edges_begin();
        eit != dt.finite_edges_end();
        ++eit )
  {
    const Eigen::Vector3d& ref_mean1 = reference_frame->kp_means[ matches[ eit->first->vertex( eit->first->cw( eit->second) )->info()].queryIdx ];
    const Eigen::Vector3d& ref_mean2 = reference_frame->kp_means[ matches[ eit->first->vertex( eit->first->ccw( eit->second) )->info()].queryIdx ];
    const Eigen::Matrix3d& ref_cov1  = reference_frame->kp_covariances[ matches[ eit->first->vertex( eit->first->cw( eit->second) )->info()].queryIdx ];
    const Eigen::Matrix3d& ref_cov2  = reference_frame->kp_covariances[ matches[ eit->first->vertex( eit->first->ccw( eit->second) )->info()].queryIdx ];

    Eigen::Vector3d ref_diff = ref_mean1 - ref_mean2 ;
    //欧氏距离
    double ref_euc_dist = ref_diff.norm();

    //马氏距离
    Eigen::Matrix3d ref_sum_cov_inv = (ref_cov1 + ref_cov2).inverse();
    Eigen::Matrix<double,1,1> ref_mah_mat = ref_diff.transpose() * ref_sum_cov_inv * ref_diff;
    double ref_mah_dist = sqrt(ref_mah_mat(0,0));

    //目标帧
    const Eigen::Vector3d& tar_mean1 = target_frame->kp_means[ matches[ eit->first->vertex( eit->first->cw( eit->second) )->info()].trainIdx ];
    const Eigen::Vector3d& tar_mean2 = target_frame->kp_means[ matches[ eit->first->vertex( eit->first->ccw( eit->second) )->info()].trainIdx ];
    const Eigen::Matrix3d& tar_cov1  = target_frame->kp_covariances[ matches[ eit->first->vertex( eit->first->cw( eit->second) )->info()].trainIdx ];
    const Eigen::Matrix3d& tar_cov2  = target_frame->kp_covariances[ matches[ eit->first->vertex( eit->first->ccw( eit->second) )->info()].trainIdx ];

    Eigen::Vector3d tar_diff = tar_mean1 - tar_mean2 ;
    //欧氏距离
    double tar_euc_dist = tar_diff.norm();

    //马氏距离
    Eigen::Matrix3d tar_sum_cov_inv = (tar_cov1 + tar_cov2).inverse();
    Eigen::Matrix<double,1,1> tar_mah_mat = tar_diff.transpose() * tar_sum_cov_inv * tar_diff;
    double tar_mah_dist = sqrt(tar_mah_mat(0,0));

    //最终的欧氏距离差
    double diff_euc_dist = fabs(tar_euc_dist - ref_euc_dist);

    //最终的马氏距离差
    double diff_mah_dist = fabs(tar_mah_dist - ref_mah_dist);


    //条件
    if( diff_mah_dist < max_connected_domain_mahalanobis_threshold
        && diff_euc_dist < max_connected_domain_threshold  )
    {
      int vex1 = eit->first->vertex( eit->first->cw( eit->second) )->info();
      int vex2 = eit->first->vertex( eit->first->ccw( eit->second))->info();
      boost::add_edge( vex1, vex2, G );
      points_inlier.push_back( std::make_pair( vex1,vex2 ) );
      points_inlier.push_back( std::make_pair( vex2,vex1 ) );
    }
  }


  std::vector<int> component( boost::num_vertices(G) );
  int num = boost::connected_components( G, &component[0] );
  std::cout << "Total number of components: " << num << std::endl;
  
  std::vector< std::vector<int> > vec_component( num );
  for ( size_t i=0; i != component.size(); ++i )
  {
    vec_component[ component[i] ].push_back(i);
  }
  
  std::sort( vec_component.begin(),vec_component.end(), vecSizeCompare );

  final_domain = vec_component[0];

  std::cout << "vec_component[0].size:  " << vec_component[0].size() << std::endl;


  if( num> 1  && vec_component[1].size() > 10 )
    {
      std::cout << "vec_component[1].size:  " << vec_component[1].size() << std::endl;
      if(areaOfDomain( vec_component[1] ) > areaOfDomain( final_domain ) )
      	final_domain = vec_component[1];
    }
  if( num> 2  &&  vec_component[2].size() > 10 )
    {
      std::cout << "vec_component[2].size:  " << vec_component[2].size() << std::endl;
			if(areaOfDomain(vec_component[2]) > areaOfDomain(final_domain))
      	final_domain = vec_component[2];
    }
  
// std::cout << "输出１" << std::endl;

//-----------------------------------------------------------
//---------------画出最终的截取结果图-------------------------
  std::cout << "final_domain.size:  " << final_domain.size() << std::endl;
 // std::cout << "输出2" << std::endl;
  std::cout << "final_domain.area:  " << areaOfDomain(final_domain) << std::endl;
// std::cout << "输出3" << std::endl;
#ifdef  MATCH_SELECTED_IMAGE
  cv::drawKeypoints( reference_frame->rgb_img , reference_frame->keypoints, tri, cv::Scalar(0,255,0));

  for ( eit = dt.finite_edges_begin(); 
        eit != dt.finite_edges_end(); 
        ++eit )
  { 
    int ind1 = eit->first->vertex( eit->first->cw( eit->second) )->info();
    int ind2 = eit->first->vertex( eit->first->ccw( eit->second))->info();

    if( find(final_domain.begin(),final_domain.end(),ind1 ) 
        == final_domain.end() )
      continue;
    if( find(final_domain.begin(),final_domain.end(),ind2) 
        == final_domain.end() )
      continue;

    //条件
    if( find(points_inlier.begin(),points_inlier.end(),std::make_pair(ind1,ind2) ) 
        != points_inlier.end() )
    { 
      cv::Point2f pt1 = 
        reference_frame->keypoints[ matches[ eit->first->vertex( eit->first->cw( eit->second) )->info()].queryIdx ].pt;
      cv::Point2f pt2 = 
        reference_frame->keypoints[ matches[ eit->first->vertex( eit->first->ccw( eit->second))->info()].queryIdx ].pt;
      cv::line( tri, pt1, pt2, cv::Scalar(0,0,255));
    }
  }
  tri.push_back(target_frame->rgb_img);

  cv::imshow("tri", tri);
  cv::waitKey(0);  
#endif
//-------------------------------------------------------------

  nums_triangle = final_domain.size();

}