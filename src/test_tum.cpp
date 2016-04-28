
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <string>

#include "opencv2/opencv.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

//自己写的头文件
#include "read_parameter.hpp"
#include "camera_intrinsics.hpp"
#include "visual_odometry.hpp"
#include "timer.hpp"

// #define dbg(...) do { fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); } while(0)
#define dbg(...)
#define info(...) do { fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); } while(0)
#define err(...) do { fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); } while(0)

#ifndef   KEY_FRAME
#define   KEY_FRAME
#endif

struct TimestampFilename
{
  TimestampFilename(int64_t ts, const std::string& fname) :
    timestamp(ts), filename(fname)
  {}
  int64_t timestamp;
  std::string filename;
};

static std::vector<TimestampFilename> read_file_index(const std::string& indexFile)
{
  FILE* fp = fopen(indexFile.c_str(), "r");
  std::vector<TimestampFilename> result;
  if(!fp) {
    perror("fopen");
    return result;
  }
  char linebuf[1024];
  int linenum = 0;
  while(!feof(fp)) {
    if(!fgets(linebuf, sizeof(linebuf), fp))
      break;
    linenum++;
    if(strlen(linebuf) == 0)
      break;
    if(linebuf[0] == '#')
      continue;
    long seconds;
    long microseconds;
    char png_fname[1024];
    if(3 != sscanf(linebuf, "%ld.%ld %s", &seconds, &microseconds, png_fname)) 
    {
      err("%s:%d Parse error", indexFile.c_str(), linenum);
      fclose(fp);
      return result;
    }
    int64_t timestamp = seconds * 1000000 + microseconds;
    result.push_back(TimestampFilename(timestamp, png_fname));
  }
  return result;
}



int main(int argc, char **argv)
{
  std::cout << "_____________________________________"
            << "parameter initialize ________________" 
            << std::endl;
  std::string camera_id = 
      getStringFromYML( "../parameter.yml", "camera_id" );
  bool visualize = getIntFromYML( "../parameter.yml", "visualize" );

  std::string datadir = 
      getStringFromYML( "../parameter.yml", "datadir" );
  std::string outdir = 
      getStringFromYML( "../parameter.yml", "outdir" );

  CameraIntrinsicsParameters camParams;
  camParams.factor = 5000;
  camParams.width = 640;
  camParams.height = 480;

  // RGB camera parameters
  if(camera_id == "fr1") {
    camParams.fx = 517.306408;
    camParams.fy = 516.469215;
    camParams.cx = 318.643040;
    camParams.cy = 255.313989;
    camParams.k1 = 0.262383;
    camParams.k2 = -0.953104;
    camParams.p1 = -0.005358;
    camParams.p2 = 0.002628;
    camParams.k3 = 1.163314;
  } else if(camera_id == "fr2") {
    camParams.fx = 520.908620;
    camParams.fy = 521.007327;
    camParams.cx = 325.141442;
    camParams.cy = 249.701764;
    camParams.k1 = 0.231222;
    camParams.k2 = -0.784899;
    camParams.p1 = -0.003257;
    camParams.p2 = -0.000105;
    camParams.k3 =  0.917205;
  } else if(camera_id == "fr3") {
    camParams.fx = 537.960322;
    camParams.fy = 539.597659;
    camParams.cx = 319.183641;
    camParams.cy = 247.053820;
    camParams.k1 = 0.026370;
    camParams.k2 = -0.100086;
    camParams.p1 = 0.003138;
    camParams.p2 = 0.002421;
    camParams.k3 = 0.000000;
  } else {
    err("Unknown camera id [%s]", camera_id.c_str());
    return 1;
  }

  // create the output trajectory file
  std::string traj_fname = outdir + "/traj.txt";
  FILE* traj_fp = fopen(traj_fname.c_str(), "w");
  if(!traj_fp)
  {
    err("Unable to create %s - %s", traj_fname.c_str(), strerror(errno));
    return 1;
  }
  std::string img_save_dir = outdir + "/result-image";

  // read the RGB and depth index files
  std::vector<TimestampFilename> rgb_fnames =
    read_file_index(datadir + "/rgb.txt");
  std::vector<TimestampFilename> depth_fnames =
    read_file_index(datadir + "/depth.txt");
  unsigned int depth_fname_index = 0;

  std::cout << std::endl
            << "_____________before synchronization＿＿＿＿＿＿＿＿ " 
            << std::endl;
  std::cout << "rgb_fnames.size: " << rgb_fnames.size() << std::endl;
  std::cout << "depth_fnames.size: " << depth_fnames.size() << std::endl;

  // synchronization
  std::vector<TimestampFilename> syn_rgb_fnames;
  std::vector<TimestampFilename> syn_depth_fnames;

  for(unsigned int rgb_fname_index=0;
      rgb_fname_index < rgb_fnames.size();
      rgb_fname_index++ )
  {
    int64_t timestamp = rgb_fnames[rgb_fname_index].timestamp;
    std::string rgb_fname =
      datadir + "/" + rgb_fnames[rgb_fname_index].filename;
    long long ts_sec = timestamp / 1000000;
    long ts_usec = timestamp % 1000000;
    char ts_str[80];
    snprintf(ts_str, 80, "%lld.%06ld", ts_sec, ts_usec);

    // match the RGB image up with a depth image
    bool matched_depth_image = false;
    while( depth_fname_index < depth_fnames.size() )
    {
      int64_t depth_ts = depth_fnames[depth_fname_index].timestamp;

      // declare a depth image match if the depth image timestamp is within
      // 40ms of the RGB image.
      int64_t dt_usec = depth_ts - timestamp;
      if(dt_usec > 20000) {
        dbg("  stop %lld.%06ld (dt %f)",
            (long long)(depth_ts / 1000000),
            (long)(depth_ts % 1000000),
            dt_usec / 1000.);
        break;
      } else if(dt_usec < -20000) {
        dbg("  skip %lld.%06ld (dt %f)",
            (long long)(depth_ts / 1000000),
            (long)(depth_ts % 1000000),
            dt_usec / 1000.);
        depth_fname_index++;
      } else {
        matched_depth_image = true;
        dbg("  mtch %lld.%06ld (dt %f)",
            (long long)(depth_ts / 1000000),
            (long)(depth_ts % 1000000),
            dt_usec / 1000.);
        break;
      }
    }

    if(!matched_depth_image) 
    {
      // didn't find a depth image with a close enough timestamp.  Skip this RGB image.
      info("# skip %s", ts_str);
      continue;
    }

    std::string depth_fname =
        datadir + "/" + depth_fnames[depth_fname_index].filename;
    depth_fname_index++;

    syn_rgb_fnames.push_back( TimestampFilename(timestamp, rgb_fname) );
    syn_depth_fnames.push_back( TimestampFilename(timestamp, depth_fname) );
  }

  std::cout << "______after synchronization＿＿＿＿＿＿＿＿ " 
            << std::endl;
  std::cout << "syn_rgb_fnames.size: " << syn_rgb_fnames.size() << std::endl;
  std::cout << "syn_depth_fnames.size: " << syn_depth_fnames.size() << std::endl<<std::endl;



//---------------------------------------------------------------------
//----------------------------------------------------------------------
  //开始视觉里程计的计算
  VisualOdometry odom( camParams );

  // 视觉里程计初始化
  std::cout << std::endl
            << "_______________________________"
            << " initialization"
            << "_______________________________"
            << std::endl;

  int64_t timestamp0 = syn_rgb_fnames[0].timestamp;
  long long ts_sec0 = timestamp0 / 1000000;
  long ts_usec0 = timestamp0 % 1000000;

  cv::Mat rgb0 = cv::imread( syn_rgb_fnames[0].filename );
  cv::Mat depth0 = cv::imread( syn_depth_fnames[0].filename, cv::IMREAD_UNCHANGED);
  odom.initialize( rgb0, depth0 );

  Eigen::Isometry3d cam_to_local0 = odom.getPose();
  Eigen::Vector3d t0 = cam_to_local0.translation();
  Eigen::Quaterniond q0(cam_to_local0.rotation());

  printf("%lld.%06ld %f %f %f %f %f %f %f\n",
      ts_sec0, ts_usec0, t0.x(), t0.y(), t0.z(), q0.x(), q0.y(), q0.z(), q0.w());
  fprintf(traj_fp, "%lld.%06ld %f %f %f %f %f %f %f\n",
      ts_sec0, ts_usec0, t0.x(), t0.y(), t0.z(), q0.x(), q0.y(), q0.z(), q0.w());

//-----------初始化结束----------------


  //统计计算失败的帧数
  int falied_frame_count(0);
  std::vector<int>  vecInliersNum;
  std::vector<int>  vecInitMatchNum,
                    vecnoRepeatMatchNum,
                    vecMatchNum;


  for( size_t index=1;
       index < syn_rgb_fnames.size();
       // index < 2;
       index++ )
  {
    int64_t timestamp = syn_rgb_fnames[index].timestamp;
    long long ts_sec = timestamp / 1000000;
    long ts_usec = timestamp % 1000000;
    char ts_str[80];
    snprintf(ts_str, 80, "%lld.%06ld", ts_sec, ts_usec);

    timer("read image");
    cv::Mat rgb = cv::imread( syn_rgb_fnames[index].filename );
    cv::Mat depth = cv::imread( syn_depth_fnames[index].filename, cv::IMREAD_UNCHANGED );
    timer("read image");

    // process the frame
    timer("process");
    odom.processFrame( rgb, depth );
    timer("process");

    MotionEstimateStatusCode status = odom.getStatus();
    switch(status) {
      case INSUFFICIENT_INLIERS:
        printf("# %s - insufficient inliers\n", ts_str);
        break;
      case OPTIMIZATION_FAILURE:
        printf("# %s - optimization failed\n", ts_str);
        break;
      case REPROJECTION_ERROR:
        printf("# %s - reprojection error too high\n", ts_str);
        break;
      case NO_DATA:
        printf("# %s - no data\n", ts_str);
        break;
      case  MOTION_TOO_LARGE:
        printf("# %s - motion too large\n", ts_str);
        break;
      case SUCCESS:
        printf("# %s - success\n", ts_str);
        break;
      default:
        break;
    }

    if( status == FAILED )
    {
      info( "\nthis frame is failed\n" );
      falied_frame_count++;
      // cv::imshow("vec_tri_domain", tri);
      // cv::imshow("match", matchss );
      // cv::waitKey(0);
    }
    else
    {
      // get the integrated pose estimate.
      Eigen::Isometry3d cam_to_local = odom.getPose();
      Eigen::Vector3d t = cam_to_local.translation();
      Eigen::Quaterniond q( cam_to_local.rotation() );

      printf("%lld.%06ld %f %f %f %f %f %f %f\n",
          ts_sec, ts_usec, t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());
      fprintf(traj_fp, "%lld.%06ld %f %f %f %f %f %f %f\n",
          ts_sec, ts_usec, t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());

      if(visualize)
        odom.save( img_save_dir );

      vecInliersNum.push_back( odom.getNumOfInliers() );

      vecInitMatchNum.push_back(odom.getNumInitMatches());
      vecnoRepeatMatchNum.push_back(odom.getNumNoRepeatMatches());
      vecMatchNum.push_back(odom.getNumMatches());
    }

  }

  info("\nLoading data from: [%s]\n", datadir.c_str());
  info("Total frame numble: [%d]", int(syn_rgb_fnames.size()) );

  fclose(traj_fp);

  timer_print_stats();


//输出调试结果
  std::cout << "max　InitMatchNum: " << *std::max_element(vecInitMatchNum.begin(),vecInitMatchNum.end()) << std::endl;
  std::cout << "min　InitMatchNum: " << *std::min_element(vecInitMatchNum.begin(),vecInitMatchNum.end()) << std::endl;
  std::cout << "avg　InitMatchNum: " 
            << (double)(std::accumulate(vecInitMatchNum.begin(), vecInitMatchNum.end(),0))/vecInitMatchNum.size()
            << std::endl;

  std::cout << "max　noRepeatMatchNum: " << *std::max_element(vecnoRepeatMatchNum.begin(),vecnoRepeatMatchNum.end()) << std::endl;
  std::cout << "min　noRepeatMatchNum: " << *std::min_element(vecnoRepeatMatchNum.begin(),vecnoRepeatMatchNum.end()) << std::endl;
  std::cout << "avg　noRepeatMatchNum: " 
            << (double)(std::accumulate(vecnoRepeatMatchNum.begin(), vecnoRepeatMatchNum.end(),0))/vecnoRepeatMatchNum.size()
            << std::endl;

  std::cout << "max　MatchNum: " << *std::max_element(vecMatchNum.begin(),vecMatchNum.end()) << std::endl;
  std::cout << "min　MatchNum: " << *std::min_element(vecMatchNum.begin(),vecMatchNum.end()) << std::endl;
  std::cout << "avg　MatchNum: " 
            << (double)(std::accumulate(vecMatchNum.begin(), vecMatchNum.end(),0))/vecMatchNum.size()
            << std::endl;

  std::cout << "max　inliers: " << *std::max_element(vecInliersNum.begin(),vecInliersNum.end()) << std::endl;
  std::cout << "min　inliers: " << *std::min_element(vecInliersNum.begin(),vecInliersNum.end()) << std::endl;
  std::cout << "avg　inliers: " 
            << (double)(std::accumulate(vecInliersNum.begin(), vecInliersNum.end(),0))/vecInliersNum.size()
            << std::endl;

#ifdef KEY_FRAME
  std::cout << "max　vec_key: " << *std::max_element(odom.vec_key.begin(),odom.vec_key.end()) << std::endl;
  std::cout << "min　vec_key: " << *std::min_element(odom.vec_key.begin(),odom.vec_key.end()) << std::endl;
  std::cout << "avg　vec_key: " 
            << (double)(std::accumulate(odom.vec_key.begin(), odom.vec_key.end(),0))/odom.vec_key.size()
            << std::endl;
#endif
            
  std::cout << "falied_frame_count: " 
            << falied_frame_count
            << std::endl;


  std::cout << odom._estimator->trans_count
            << std::endl
            << odom._estimator->angle_count
            << std::endl
            << odom._estimator->inliers_count
            << std::endl;

  return 0;
}
