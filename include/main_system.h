#ifndef SYSTEM_H
#define SYSTEM_H

#include<string>
#include<thread>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "tracking.h"

namespace MONOCULAR_ORB_SLAM2
{

class System
{
public:

public:
    System(const std::string &strSettingsFile, const bool bUseViewer = true);
	cv::Mat track_monocular(const cv::Mat& im, const double& timestamp);

private:
	Tracking* m_tracker;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
