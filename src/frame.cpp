#include "frame.h"

namespace MONOCULAR_ORB_SLAM2
{
	float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;

	Frame::Frame()
	{
	}

	Frame::Frame(const cv::Mat& imGray, const double& timeStamp, int feature_num, cv::Mat& K, cv::Mat& distCoef)
		: mTimeStamp(timeStamp), mK(K.clone()), mDistCoef(distCoef.clone())
	{
		extract_orb(imGray, feature_num);

		num_keypoints = m_keypoints.size();
		if (m_keypoints.empty())
			return;
	}

	void Frame::extract_orb(const cv::Mat& v_img,int feature_num) {
		auto detector = cv::ORB::create(feature_num);
		detector->detectAndCompute(v_img, cv::noArray(), m_keypoints, m_descriptors);
	}
} 
