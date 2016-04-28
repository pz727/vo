
#ifndef __visual_odometry_frame_hpp__
#define __visual_odometry_frame_hpp__

#include <Eigen/Geometry>

#include "opencv2/opencv.hpp"
#include <opencv2/xfeatures2d/nonfree.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "camera_intrinsics.hpp"
#include "timer.hpp"
#include "read_parameter.hpp"


typedef pcl::PointXYZRGB              PointT;
typedef pcl::PointCloud<PointT>       PointCloudT;

class Frame
{
  public:

    Frame( CameraIntrinsicsParameters * input_camera )
    {
      pCamera = input_camera;

      // cv::initModule_nonfree();
       
      // _sift = cv::xfeatures2d::SIFT::create(1000);
      _surf = cv::xfeatures2d::SURF::create( 1000 );

      // _orb = cv::ORB::create ( 1500, 1.2, 4 );

      // cv::String detector_string = getStringFromYML("../parameter.yml", "detector");
      // _detector = cv::FeatureDetector::create(detector_string);

      // cv::String descriptor_string = getStringFromYML("../parameter.yml", "descriptor");
      // _descriptor = cv::DescriptorExtractor::create(descriptor_string);

    }


    ~Frame()
    {
    }

    void prepareFrame(const cv::Mat& rgb, const cv::Mat& depth);

    void gridFilter( std::vector<cv::KeyPoint>& keyp, int u, int v, int total_number );
    // static bool load( Frame* frame, const cv::String& path );

    cv::Mat                    rgb_img, depth_img;
    std::vector<cv::KeyPoint>  keypoints;
    std::vector<cv::Point3f>   point_xyz;
    cv::Mat                    descriptors;

    std::vector<Eigen::Vector3d>  kp_means;
    std::vector<Eigen::Matrix3d>  kp_covariances;
    
  private:

    double getStdDevZ(double z) const;

    double getVarZ(double z) const;

    void getGaussianDistribution(int u, int v, double& z_mean, double& z_var) const;

    void getGaussianMixtureDistribution(int u, int v, double& z_mean, double& z_var) const;

    //计算其分布
    void computeDistributions( double max_z = 5.5, double max_stdev_z = 0.03 );


    // void constructFeaturePointCloud( PointCloudFeature& cloud );

    void constructDensePointCloud(PointCloudT& cloud,
                                  double max_z = 5.5,
                                  double max_stdev_z = 0.03) const;

    void drawDepthUncertainty( double max_z = 5.5,
                               double max_stdev_z = 0.03);



    //摄像机内参数
    CameraIntrinsicsParameters * pCamera;

    //特征检测与描述类
    cv::Ptr<cv::xfeatures2d::SIFT>      _sift;
    cv::Ptr<cv::xfeatures2d::SURF>      _surf;
    cv::Ptr<cv::ORB>                    _orb;
    // cv::Ptr<cv::Feature2D>      _detector;
    // cv::Ptr<cv::DescriptorExtractor>     _descriptor;
    // cv::SIFT        sift;

    cv::KeyPointsFilter    keyPointsFilter;

    //图像及点云
    cv::Mat       depth_uncertainty;
    PointCloudT   cloud_one_frame;

    //检测到的关键点，cv格式的３Ｄ点，描述子
    std::vector<cv::KeyPoint>  init_keypoints;


    static const double         Z_STDEV_CONSTANT = 0.001425;
    // static constexpr double         Z_STDEV_CONSTANT = 0.001425;

    int n_valid_keypoints;

//  大体问题不大了，唯一存在的疑惑是深度值，
//  没有插值，也没有四舍五入，而是直接取整

};

#endif
