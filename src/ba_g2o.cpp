
#include "ba_g2o.hpp"


void BA_G2O::ba_Optimal(
        								Frame*  reference_frame,
                        Frame*  target_frame,
      std::vector<cv::DMatch>&  vecMatches,
      		 Eigen::Isometry3d &  trans )
{
  unsigned int index(0);
	optimizer.clear();

	//第一个pose
	g2o::VertexSE3Expmap * pos1 = new g2o::VertexSE3Expmap();//相机位姿表示
	pos1->setId( index++ );
	pos1->setFixed( true ); // 第一个点固定为零
  pos1->setEstimate( g2o::SE3Quat() );
  optimizer.addVertex( pos1 );

  //第二个pose
	g2o::VertexSE3Expmap * pos2 = new g2o::VertexSE3Expmap();//相机位姿表示
	pos2->setId( index++ );
  pos2->setEstimate( g2o::SE3Quat( Eigen::Quaterniond( trans.rotation() ),
                                   g2o::Vector3D( trans.translation() )     ));
  optimizer.addVertex( pos2 );

	for ( auto match:vecMatches ) //c++11新特性
	{
		// 添加3-d　point　节点
		g2o::VertexSBAPointXYZ * point_3d = new g2o::VertexSBAPointXYZ();
		point_3d->setId( index++ );
		point_3d->setMarginalized(true);//边缘化。没看懂什么意思
    point_3d->setEstimate( reference_frame->kp_means[ match.queryIdx ] );
    optimizer.addVertex( point_3d );

    //添加约束，第一帧
    g2o::EdgeProjectXYZ2UV*  edge1 = new g2o::EdgeProjectXYZ2UV();
    edge1->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>  (optimizer.vertex(index-1) ));
		edge1->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>    (optimizer.vertex(0)       ));      
    //量测值
    edge1->setMeasurement( Eigen::Vector2d( reference_frame->keypoints[match.queryIdx].pt.x, reference_frame->keypoints[match.queryIdx].pt.y) );
    //系统信息矩阵，或者加权信息
    edge1->setInformation( Eigen::Matrix2d::Identity() );
    edge1->setParameterId(0, 0);//这个参数有什么用呢  
    // 核函数,设置鲁棒核Huber
    edge1->setRobustKernel( new g2o::RobustKernelHuber() );
    optimizer.addEdge( edge1 );
    // edges.push_back( edge1 );

    //添加约束,第二帧
    g2o::EdgeProjectXYZ2UV*  edge2 = new g2o::EdgeProjectXYZ2UV();
		edge2->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(index-1)  ));
		edge2->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(1) ));
    //量测值
    edge2->setMeasurement( Eigen::Vector2d( target_frame->keypoints[match.trainIdx].pt.x, target_frame->keypoints[match.trainIdx].pt.y) );
    //系统信息矩阵，或者加权信息
    edge2->setInformation( Eigen::Matrix2d::Identity() );
    edge2->setParameterId(0, 0);//这个参数有什么用呢  
    // 核函数,设置鲁棒核Huber
    edge2->setRobustKernel( new g2o::RobustKernelHuber() );
    optimizer.addEdge( edge2 );
    // edges.push_back( edge );    
	}
       
  timer("initialize");
  optimizer.initializeOptimization();
  timer("initialize");

  timer("optimize");
  //终止条件达到后，自动停止迭代
  optimizer.optimize( num_iteration );
  timer("optimize");


  //我们比较关心两帧之间的变换矩阵
  g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*> ( optimizer.vertex(1) );
  trans = v->estimate();


    //     // 以及所有特征点的位置
    // for ( size_t i=0; i<pts1.size(); i++ )
    // {
    //     g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+2));
    //     cout<<"vertex id "<<i+2<<", pos = ";
    //     Eigen::Vector3d pos = v->estimate();
    //     cout<<pos(0)<<","<<pos(1)<<","<<pos(2)<<endl;
    // }

    // 估计inlier的个数
    // int inliers = 0;
    // for ( auto e:edges ) //c++11新特性
    // {
    //     e->computeError();
    //     // chi2 就是 error*\Omega*error, 如果这个数很大，说明此边的值与其他边很不相符
    //     if ( e->chi2() > 1 )//重投影误差值平方，大于１个像素值
    //     {
    //         cout<<"error = "<<e->chi2()<<endl;
    //     }
    //     else 
    //     {
    //         inliers++;
    //     }
    // }
}