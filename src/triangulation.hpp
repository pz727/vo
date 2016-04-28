
#ifndef __Feature_Matcher_Trianglation_hpp__
#define __Feature_Matcher_Trianglation_hpp__

#include <math.h> //用于fabs函数

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_data_structure_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Polygon_2.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Projection_traits_xy_3<K>  Gt;
typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned, Gt> Vb;
typedef CGAL::Triangulation_data_structure_2<Vb> Tds;
typedef CGAL::Delaunay_triangulation_2< Gt,Tds > Delaunay;
typedef CGAL::Polygon_2<Gt> Polygon_2;
typedef K::Point_3  Point;

#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> Graph;

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "frame.hpp"
#include "read_parameter.hpp"


class MaxConnectedDomain
{
	public:
		friend class FeatureMatcher;

		MaxConnectedDomain()
    {
			max_connected_domain_threshold = 
				getDoubleFromYML("../parameter.yml", "max_connected_domain_threshold");

			max_connected_domain_mahalanobis_threshold = 
			  getDoubleFromYML("../parameter.yml", "max_connected_domain_mahalanobis_threshold");
		}

		~MaxConnectedDomain()
		{}

		void computeMaxConnectedDomain(
            Frame* reference_frame,
            Frame* target_frame,
            std::vector<cv::DMatch>& matches);


		double areaOfDomain( const std::vector<int> &ca );

	private:




		//最大连接域算法，参数
	  std::vector< std::pair< Point,int > > points;

		double max_connected_domain_threshold;
	  double max_connected_domain_mahalanobis_threshold;

	  std::vector< std::pair<int,int> > points_inlier;

	  int 		nums_triangle;
		std::vector<int> 	final_domain;

	  cv::Mat     img_original_triangulation_mesh;
		cv::Mat    	tri;
};

#endif

