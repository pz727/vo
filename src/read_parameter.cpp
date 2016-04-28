
#include "read_parameter.hpp"

#include <iostream>

int getIntFromYML( const cv::String& file_name, const cv::String parameter_name)
{
	cv::FileStorage fs( file_name, cv::FileStorage::READ );
	int result = (int) fs[parameter_name];
	fs.release();
	std::cout << parameter_name << ": " << result << std::endl;
	return result;
}

double getDoubleFromYML( const cv::String& file_name, const cv::String parameter_name)
{
	cv::FileStorage fs( file_name, cv::FileStorage::READ );
	double result = (double) fs[parameter_name];
	fs.release();
	std::cout << parameter_name << ": " << result << std::endl;
	return result;
}

cv::String getStringFromYML( const cv::String& file_name, const cv::String parameter_name)
{
	cv::FileStorage fs( file_name, cv::FileStorage::READ );
	cv::String result;
    fs[parameter_name] >> result;
    fs.release();
    std::cout << parameter_name << ": " << result << std::endl;
	return result;
}
