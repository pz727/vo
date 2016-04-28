
cmake_minimum_required(VERSION 2.8)
project( vo )

#设置cmake模式，，，虽然我也还没搞懂
set( CMAKE_BUILD_TYPE Release )

#c++11特性的添加  -Wall
# set( CMAKE_CXX_FLAGS "-std=c++11 -O2 -march=native" )

#加入cmake模块文件夹
# list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

#opencv
find_package( OpenCV REQUIRED )
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")


#pcl点云库
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#CGAL，计算机图形算法库
find_package( CGAL QUIET COMPONENTS )
if ( CGAL_FOUND )
  include( ${CGAL_USE_FILE} )
endif()

# eigen　，这个库只需添加头文件就行
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

#以下３个都是g2o库，cholmod与csparse选一个就好
# find_package( G2O )
# if(G2O_FOUND)
# 	include_directories( ${G2O_INCLUDE_DIR} )
# else()
#   MESSAGE(STATUS "Not found G2O and its dependencies")
# endif()
# find_package( Cholmod )
# if( CHOLMOD_FOUND )
# 	include_directories( ${CHOLMOD_INCLUDE_DIR} )
# else()
#   MESSAGE(STATUS "Not found CHOLMOD and its dependencies")
# endif()
# find_package( CSparse )
# if(CSPARSE_FOUND)
# 	include_directories( ${CSPARSE_INCLUDE_DIR} )
# else()
#   MESSAGE(STATUS "Not found CSPARSE and its dependencies")
# endif()


set(SRC_List 
	src/test_tum.cpp
	# src/ba_g2o.cpp
	src/read_parameter.cpp
	src/timer.cpp
	src/frame.cpp
	src/motion_estimate.cpp
	src/visual_odometry.cpp
	src/feature_matcher.cpp
	src/triangulation.cpp
	)

add_executable( vo  ${SRC_List} )

target_link_libraries( 	vo 
			${OpenCV_LIBS} 
		 	${PCL_LIBRARIES}

		 	#　g2o链接
	    # ${CHOLMOD_LIBRARIES}
	    # ${CSPARSE_LIBRARY}
	    # ${G2O_CORE_LIBRARY} 
	    # ${G2O_TYPES_SLAM3D}
	    # ${G2O_SOLVER_CSPARSE}
	    # ${G2O_STUFF_LIBRARY}
	    # ${G2O_SOLVER_CSPARSE_EXTENSION}
	    # ${G2O_TYPES_SBA}
					 )
