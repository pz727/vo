#ifndef __g2o_bundle_ajustment_hpp__
#define __g2o_bundle_ajustment_hpp__

#define CHOLMOD
// #define CSPARSE

// for std
#include <iostream>

// for opencv 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <boost/concept_check.hpp>

// for g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "timer.hpp"
#include "camera_intrinsics.hpp"
#include "frame.hpp"

class BA_G2O
{
  public:
    friend class MotionEstimator;

    BA_G2O( CameraIntrinsicsParameters* input_camera )
    {
      g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
    #ifdef CHOLMOD
      std::cout << "Using CHOLMOD" << std::endl;
      linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();
    #endif
    #ifdef CSPARSE
      linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>();
      std::cout << "Using CSPARSE" << std::endl;
    #endif
      // 6*3 的参数
      g2o::BlockSolver_6_3* block_solver = new g2o::BlockSolver_6_3( linearSolver );
      // L-M 下降 
      g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg( block_solver );
    
      optimizer.setAlgorithm( algorithm );
      optimizer.setVerbose( false );

      // 准备相机参数
      g2o::CameraParameters* camera = new g2o::CameraParameters( input_camera->fx, Eigen::Vector2d(input_camera->cx, input_camera->cy), 0);
      camera->setId(0);
      optimizer.addParameter( camera );

      num_iteration = getIntFromYML("../parameter.yml", "num_iteration_g2o");

    }

    ~BA_G2O()
    {}

    void ba_Optimal(
        std::vector< Frame * >&                 vecFrame,
        std::vector< std::vector<cv::DMatch> >& vecMatches,
        std::vector< Eigen::Isometry3d >&       vecPose );

    void ba_Optimal(    Frame*   reference_frame,
                        Frame*   target_frame,
      std::vector<cv::DMatch>&   vecMatches,
            Eigen::Isometry3d &  trans );

  private:
    g2o::SparseOptimizer        optimizer;
    unsigned int                num_iteration;
};

#endif