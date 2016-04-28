#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Polygon_2.h>
#include <vector>
#include <CGAL/ch_graham_andrew.h>
#include <CGAL/Projection_traits_xy_3.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Projection_traits_xy_3<K>  Gt;

typedef K::Point_3   Point;
// typedef Gt::Point_2  Point_2;
typedef CGAL::Polygon_2< Gt > Polygon_2;



int main()
{
  std::vector<Point> points, result;
  Polygon_2 p;
  points.push_back(Point(0,0,5));
  points.push_back(Point(10,0,3));
  points.push_back(Point(10,10,0));
  points.push_back(Point(6,5,3));
  points.push_back(Point(4,1,10));
  Gt gt;
  CGAL::convex_hull_2( points.begin(), points.end(), std::back_inserter(p),gt );
  // CGAL::ch_graham_andrew( points.begin(), points.end(), std::back_inserter(result),Gt);
  std::cout << p.size() << " points on the convex hull" << std::endl;
  std::cout << p[0] << std::endl;
  std::cout << p[1] << std::endl;
  std::cout << p[2] << std::endl;

  double Area = p.area();

  std::cout << "the area of polygon p is " << Area << std::endl;

  return 0;
}