
#ifndef __Feature_Matcher_hpp__
#define __Feature_Matcher_hpp__

#include <math.h> //用于fabs函数

// #include <opencv2/features2d/features2d.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "frame.hpp"
#include "triangulation.hpp"


struct Max_clique_match
{
	int id;
	int compatibility_degree;
	cv::DMatch clique_match;
	std::vector<bool> consistency_vec;
	bool 	in_clique;
};

class FeatureMatcher
{
	public:
		FeatureMatcher( CameraIntrinsicsParameters* input_camera )
		: bf_matcher( cv::NORM_L2, true )
    {
      pCamera = input_camera;

			_maxConDom = new MaxConnectedDomain();

			max_feature_motion = getIntFromYML("../parameter.yml", "max_feature_motion");
			max_feature_distance = getIntFromYML("../parameter.yml", "max_feature_distance");
		
			clique_inlier_threshold = 
					getDoubleFromYML("../parameter.yml", "clique_inlier_threshold");

		}
		~FeatureMatcher()
		{
			delete  _maxConDom;
		}

		void match( Frame* reference_frame,
	              Frame* target_frame,
	        const Eigen::Isometry3d &init_motion_est);        

		void matchFilter(
						Frame* reference_frame,
            Frame* target_frame,
      const Eigen::Isometry3d& init_motion_est     );

		void computeMaximallyConsistentClique(
						Frame* reference_frame,
            Frame* target_frame );

		cv::Point2f point3Dto2D( cv::Point3f& point_xyz, 
    									 const Eigen::Isometry3d & trans );


    int getNumInitMatches() const 
    {
      return num_InitMatches;
    }

    int getNumNoRepeatMatches() const 
    {
      return num_NoRepeatMatches;
    }

    int getNumMatches() const 
    {
      return num_Matches;
    }

	// private:
		
		CameraIntrinsicsParameters* pCamera;

		cv::BFMatcher bf_matcher;

		MaxConnectedDomain* _maxConDom;
		
		//匹配参数
		std::vector<cv::DMatch> init_matches,
														no_repeat_matches,
														matches,  //三角剖分后的特征匹配
														final_matches;//最大团算法后的特征匹配

		int max_feature_motion;
		int max_feature_distance;

		//最大团算法，参数
		std::vector<Max_clique_match> max_clique;
		double clique_inlier_threshold;

    int  num_InitMatches,num_NoRepeatMatches,num_Matches,num_final_Matches;

    cv::Mat 	imgMatch;//最后的匹配图片
};

#endif

