
#include "frame.hpp"


void Frame::prepareFrame(const cv::Mat& rgb, const cv::Mat& depth)
{
	rgb_img = rgb.clone();
  depth_img = depth.clone();

  init_keypoints.clear();
  // init_keypoints.reserve(1500);
	keypoints.clear();
  // keypoints.reserve(1000);
	point_xyz.clear();
  // point_xyz.reserve(1000);

  timer("feature detect");
  // _sift->detect( rgb_img, init_keypoints );
  _surf->detect( rgb_img, init_keypoints );
  // _orb->detect( rgb_img, init_keypoints );
	// _detector->detect( rgb_img, init_keypoints );
  // sift( rgb_img, cv::Mat(), init_keypoints, cv::noArray() );
  timer("feature detect");

  std::cout << "init_keypoints:       "<< init_keypoints.size() << std::endl;
  // std::cout << "init_keypoints[0]:       "<< init_keypoints[0].pt.x << "  " << init_keypoints[0].pt.y << std::endl;
  // cv::Mat outImage1;
  // cv::drawKeypoints(rgb, init_keypoints, outImage1);
  // cv::imshow("keypoint", outImage1);
  // cv::waitKey(0);


  // gridFilter( init_keypoints, 4, 3, 1500 );
  // std::cout << "gridFilter_keypoints:       "<< init_keypoints.size() << std::endl;
  // cv::Mat outImage2;
  // cv::drawKeypoints(rgb, init_keypoints, outImage2);
  // cv::imshow("keypoint", outImage2);
  // cv::waitKey(0);


  computeDistributions();
  std::cout << "num_valid_keypoints:  " << n_valid_keypoints << std::endl;
  // cv::Mat outImage2;
  // cv::drawKeypoints(rgb, keypoints, outImage2);
  // cv::imshow("keypoint", outImage2);
  // cv::waitKey(0);



  // constructDensePointCloud(cloud_one_frame);
  // drawDepthUncertainty();
  // cv::imshow( "DepthUncertainty", depth_uncertainty );
  // cv::waitKey(0);
  
  timer("descriptor");
  // _sift->compute( rgb, keypoints, descriptors );
  _surf->compute( rgb, keypoints, descriptors );
  // _orb->compute( rgb, keypoints, descriptors );
  // _descriptor->compute( rgb, keypoints, descriptors );
  // sift( rgb_img, cv::Mat(), keypoints, descriptors, true );
  timer("descriptor");

}





double Frame::getStdDevZ(double z) const
{
  return Z_STDEV_CONSTANT * z * z;
}

double Frame::getVarZ(double z) const
{
  double std_dev_z = getStdDevZ(z);
  return std_dev_z * std_dev_z;
}

void Frame::getGaussianDistribution(
  int u, int v, double& z_mean, double& z_var) const
{
  // get raw z value (in mm)
  uint16_t z_raw = depth_img.at<uint16_t>(v, u);

  // z [meters]
  z_mean = z_raw / pCamera->factor ;

  // var_z [meters]
  z_var = getVarZ(z_mean);
}

void Frame::getGaussianMixtureDistribution(
  int u, int v, double& z_mean, double& z_var) const
{
  /// @todo Different window sizes? based on sigma_u, sigma_v?
  int w = 1;

  int u_start = std::max(u - w, 0);
  int v_start = std::max(v - w, 0);
  int u_end   = std::min(u + w, depth_img.cols - 1);
  int v_end   = std::min(v + w, depth_img.rows - 1);

  // iterate accross window - find mean
  double weight_sum = 0.0;
  double mean_sum   = 0.0;
  double alpha_sum  = 0.0;

  for (int uu = u_start; uu <= u_end; ++uu)
  for (int vv = v_start; vv <= v_end; ++vv)
  {
    uint16_t z_neighbor_raw = depth_img.at<uint16_t>(vv, uu);
 
    if (z_neighbor_raw != 0)  //只处理了深度值存在的情况
    {
      double z_neighbor = z_neighbor_raw / pCamera->factor ;

      // determine and aggregate weight
      double weight;
      if       (u==uu && v==vv) weight = 4.0;
      else if  (u==uu || v==vv) weight = 2.0;
      else                      weight = 1.0; 
      weight_sum += weight;

      // aggregate mean
      mean_sum += weight * z_neighbor;

      // aggregate var
      double var_z_neighbor = getVarZ(z_neighbor);
      alpha_sum += weight * (var_z_neighbor + z_neighbor * z_neighbor);
    }
  }

  z_mean = mean_sum  / weight_sum;
  z_var  = alpha_sum / weight_sum - z_mean * z_mean;
}


void Frame::computeDistributions(
  double max_z,
  double max_stdev_z)
{
  double max_var_z = max_stdev_z * max_stdev_z; // maximum allowed z variance

  /// @todo These should be arguments or const static members
  double s_u = 1.0;            // uncertainty in pixels
  double s_v = 1.0;            // uncertainty in pixels

  n_valid_keypoints = 0;

  // center point
  double cx = pCamera->cx;
  double cy = pCamera->cy;

  // focus length
  double fx = pCamera->fx;
  double fy = pCamera->fy;

  // precompute for convenience
  double var_u = s_u * s_u;
  double var_v = s_v * s_v;
  double fx2 = fx*fx;
  double fy2 = fy*fy;

  kp_means.clear();
  kp_means.reserve(1000);
  kp_covariances.clear();
  kp_covariances.reserve(1000);

  for(unsigned int kp_idx = 0; kp_idx < init_keypoints.size(); ++kp_idx)
  {
    // calculate pixel coordinates
    double u = init_keypoints[kp_idx].pt.x;
    double v = init_keypoints[kp_idx].pt.y;

    int uu_int = int(u+0.5);
    int vv_int = int(v+0.5);

    // get raw z value
    uint16_t z_raw = depth_img.at<uint16_t>( vv_int, uu_int );

    // skip bad values  
    if (z_raw == 0)
      continue;
  
    // get z: mean and variance
    double z, var_z;
    // getGaussianDistribution( uu_int, vv_int, z, var_z );
    //   u,v都舍去小数部分了
    getGaussianMixtureDistribution( uu_int, vv_int, z, var_z );

    // skip bad values - too far away, or z-variance too big
    if ( z > max_z || var_z > max_var_z )
      continue;

    n_valid_keypoints++;

    // precompute for convenience
    double z_2  = z * z;
    double umcx = u - cx;
    double vmcy = v - cy;

    // calculate x and y
    double x = z * umcx / fx;
    double y = z * vmcy / fy;
  
    // calculate covariances
    double s_xz = var_z * umcx / fx;
    double s_yz = var_z * vmcy / fy;

    double s_xx = (var_z * umcx * umcx + var_u * (z_2 + var_z))/fx2;
    double s_yy = (var_z * vmcy * vmcy + var_v * (z_2 + var_z))/fy2;

    double s_xy = umcx * vmcy * var_z / (fx * fy);
    double s_yx = s_xy;

    double s_zz = var_z; 

    double s_zy = s_yz;
    double s_zx = s_xz;
   
    // fill out mean matrix
    Eigen::Vector3d kp_mean;
    kp_mean(0,0) = x;
    kp_mean(1,0) = y;
    kp_mean(2,0) = z;

    // fill out covariance matrix
    Eigen::Matrix3d kp_covariance;
    kp_covariance(0,0) = s_xx; // xx
    kp_covariance(0,1) = s_xy; // xy
    kp_covariance(0,2) = s_xz; // xz

    kp_covariance(1,0) = s_yx; // xy
    kp_covariance(1,1) = s_yy; // yy
    kp_covariance(1,2) = s_yz; // yz

    kp_covariance(2,0) = s_zx; // xz-
    kp_covariance(2,1) = s_zy; // yz
    kp_covariance(2,2) = s_zz; // zz

    kp_means.push_back(kp_mean);
    kp_covariances.push_back(kp_covariance);

    keypoints.push_back( init_keypoints[kp_idx] );
    point_xyz.push_back( cv::Point3f(kp_mean(0), kp_mean(1), kp_mean(2)) );
  }

}


void Frame::constructDensePointCloud(
  PointCloudT&       cloud,
  double             max_z,
  double        max_stdev_z )   const
{
  double max_var_z = max_stdev_z * max_stdev_z; // maximum allowed z variance

  // center point
  double cx = pCamera->cx;
  double cy = pCamera->cy;

  // focus length
  double fx = pCamera->fx;
  double fy = pCamera->fy;

  // Scale by focal length for computing (X,Y)
  float constant_x = 1.0 / fx;
  float constant_y = 1.0 / fy;

  float bad_point = std::numeric_limits<float>::quiet_NaN();

  cloud.points.clear();
  cloud.points.resize(rgb_img.rows * rgb_img.cols);
  for (int v = 0; v < rgb_img.rows; ++v)
  for (int u = 0; u < rgb_img.cols; ++u)
  {
    unsigned int index = v * rgb_img.cols + u;

    //这里本身就是整型值，不需要考虑舍去问题
    uint16_t z_raw = depth_img.at<uint16_t>(v, u);

    float z = z_raw / pCamera->factor ; //convert to meters

    PointT& p = cloud.points[index];

    double z_mean, z_var;

    // check for out of range or bad measurements
    if (z_raw != 0)
    {
      getGaussianMixtureDistribution(u, v, z_mean, z_var);

      // check for variance and z limits     
      if (z_var < max_var_z && z_mean < max_z)
      {
        // fill in XYZ
        p.x = z * (u - cx) * constant_x;
        p.y = z * (v - cy) * constant_y;
        p.z = z;
      }
      else
      {
        p.x = p.y = p.z = bad_point;
      }
    }
    else
    {
      p.x = p.y = p.z = bad_point;
    }
 
    // fill out color
    const cv::Vec3b& color = rgb_img.at<cv::Vec3b>(v,u);
    p.r = color[2];
    p.g = color[1];
    p.b = color[0];
  }

  // // set cloud header
  // cloud.header.frame_id = header.frame_id;
  // // The point cloud stamp, in usec.
  // cloud.header.stamp = header.stamp.sec * 1e6 + header.stamp.nsec * 1e-3;
  // cloud.header.seq = header.seq;
    
  cloud.height = rgb_img.rows;
  cloud.width  = rgb_img.cols;
  cloud.is_dense = false;
}

/*bool Frame::load( Frame* frame, const cv::String& path)
{
  // set the filenames
  std::string rgb_filename    = path + "/rgb.png";
  std::string depth_filename  = path + "/depth.png";
  std::string header_filename = path + "/header.yml";
  std::string intr_filename   = path + "/intr.yml";

  // check if files exist
  if (!boost::filesystem::exists(rgb_filename)    ||
      !boost::filesystem::exists(depth_filename)  ||
      !boost::filesystem::exists(header_filename) ||
      !boost::filesystem::exists(intr_filename) )
  {
    std::cerr << "files for loading frame not found" << std::endl;
    return false;
  }

  // load header
  cv::FileStorage fs_h(header_filename, cv::FileStorage::READ);
  int seq, sec, nsec;

  fs_h["frame_id"]   >> frame.header.frame_id;
  fs_h["seq"]        >> seq;
  fs_h["stamp_sec"]  >> sec;
  fs_h["stamp_nsec"] >> nsec;

  frame.header.seq        = seq;
  frame.header.stamp.sec  = sec;
  frame.header.stamp.nsec = nsec;

  fs_h["index"] >> frame.index;

  // load images
  frame.rgb_img = cv::imread(rgb_filename);
  frame.depth_img = cv::imread(depth_filename, -1);

  // load intrinsic matrix
  cv::FileStorage fs_mat(intr_filename, cv::FileStorage::READ);
  fs_mat["intr"] >> frame.intr;

  return true;
}*/



void Frame::drawDepthUncertainty(
   double max_z,
   double max_stdev_z)
{
  depth_uncertainty = rgb_img.clone();

  double max_var_z = max_stdev_z * max_stdev_z; // maximum allowed z variance

  // center point
  double cx = pCamera->cx;
  double cy = pCamera->cy;

  // focus length
  double fx = pCamera->fx;
  double fy = pCamera->fy;

  // Scale by focal length for computing (X,Y)
  float constant_x = 1.0 / fx;
  float constant_y = 1.0 / fy;

  for (int v = 0; v < rgb_img.rows; ++v)
  { 
    uchar* data =  depth_uncertainty.ptr<uchar>(v);

    for (int u = 0; u < rgb_img.cols; ++u)
    {
      //这里本身就是整型值，不需要考虑舍去问题
      uint16_t z_raw = depth_img.at<uint16_t>(v, u);
      float z = z_raw / pCamera->factor ; //convert to meters
      double z_mean, z_var;
      
      if ( z_raw != 0 )
      {
        getGaussianMixtureDistribution(u, v, z_mean, z_var);

        if (z_var < max_var_z && z_mean < max_z)
        {
          *data++ = 0;//深度不确定度在可接受范围内
          *data++ = 0;//深度由小变大，颜色由黑变红
          *data++ = z_var / max_var_z * 255;
        }
        else
        {
          *data++ = 0;
          *data++ = 255;//深度不确定度太大的，为绿色
          *data++ = 0;
        }
      }
      else
      {
        *data++ = 255;//深度值为零的或者无效的，为蓝色
        *data++ = 0;
        *data++ = 0;
      }
    }
  }

}



void Frame::gridFilter( std::vector<cv::KeyPoint>& keyp, int u, int v, int total_number )
{
  std::vector< std::vector<cv::KeyPoint> > veckeyp( u * v );
  int index(0);
  for(size_t i=0; i<u; i++)
    for(size_t j=0; j<v; j++)
    {
      cv::Rect rect( cv::Point( 640*i/u, 480*j/v ), cv::Point( 640*(i+1)/u, 480*(j+1)/v) );
      for( size_t k=0; k < keyp.size(); k++ )
        if( rect.contains( keyp[k].pt ) )
            veckeyp[index].push_back(keyp[k]);
      keyPointsFilter.retainBest(veckeyp[index], total_number/u/v );
      index++;
    }

//清空原来的keypoints，并用过滤后的keypoints，填充
  keyp.clear();
  for( size_t k=0; k < u*v ; k++ )
    keyp.insert( keyp.end(), veckeyp[k].begin(), veckeyp[k].end() );
}

