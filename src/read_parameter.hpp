#ifndef __read_parameter_hpp__
#define __read_parameter_hpp__

#include "opencv2/opencv.hpp"

int getIntFromYML(const  cv::String& file_name, const cv::String parameter_name);

double getDoubleFromYML( const cv::String& file_name, const cv::String parameter_name);

cv::String getStringFromYML( const cv::String& file_name, const cv::String parameter_name);

#endif