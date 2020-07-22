#ifndef TRACKING_H
#define TRACKING_H
#include <opencv2/core/mat.hpp>

#include "frame.h"

namespace MONOCULAR_ORB_SLAM2
{
	class Tracking
	{
	public:
		Tracking(const std::string& strSettingPath);

		cv::Mat grab_image_monocular(const cv::Mat& im, const double& timestamp);

	public:
		enum eTrackingState
		{
			SYSTEM_NOT_READY=-1,
			NO_IMAGES_YET=0,
			NOT_INITIALIZED=1,
			OK=2,
			LOST=3
		};

		eTrackingState mState;
		eTrackingState mLastProcessedState;

	protected:
		cv::Mat mK;
		cv::Mat mDistCoef;
		float mbf;

		Frame m_current_frame;
		std::vector<Frame> frame_vector;
	};
} 

#endif // TRACKING_H
