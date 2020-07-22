#include <iostream>
#include <mutex>

#include "tracking.h"

#include <opencv2/core/persistence.hpp>

using namespace std;

namespace MONOCULAR_ORB_SLAM2
{
	Tracking::Tracking(const std::string& strSettingPath):
		mState(NO_IMAGES_YET)
	{
		// Load camera parameters from settings file
		cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
		float fx = fSettings["Camera.fx"];
		float fy = fSettings["Camera.fy"];
		float cx = fSettings["Camera.cx"];
		float cy = fSettings["Camera.cy"];

		cv::Mat K = cv::Mat::eye(3, 3,CV_32F);
		K.at<float>(0, 0) = fx;
		K.at<float>(1, 1) = fy;
		K.at<float>(0, 2) = cx;
		K.at<float>(1, 2) = cy;
		K.copyTo(mK);

		cv::Mat DistCoef(4, 1,CV_32F);
		DistCoef.at<float>(0) = fSettings["Camera.k1"];
		DistCoef.at<float>(1) = fSettings["Camera.k2"];
		DistCoef.at<float>(2) = fSettings["Camera.p1"];
		DistCoef.at<float>(3) = fSettings["Camera.p2"];
		const float k3 = fSettings["Camera.k3"];
		if (k3 != 0)
		{
			DistCoef.resize(5);
			DistCoef.at<float>(4) = k3;
		}
		DistCoef.copyTo(mDistCoef);

		mbf = fSettings["Camera.bf"];

		float fps = fSettings["Camera.fps"];
		if (fps == 0)
			fps = 30;

		cout << endl << "Camera Parameters: " << endl;
		cout << "- fx: " << fx << endl;
		cout << "- fy: " << fy << endl;
		cout << "- cx: " << cx << endl;
		cout << "- cy: " << cy << endl;
		cout << "- k1: " << DistCoef.at<float>(0) << endl;
		cout << "- k2: " << DistCoef.at<float>(1) << endl;
		if (DistCoef.rows == 5)
			cout << "- k3: " << DistCoef.at<float>(4) << endl;
		cout << "- p1: " << DistCoef.at<float>(2) << endl;
		cout << "- p2: " << DistCoef.at<float>(3) << endl;
		cout << "- fps: " << fps << endl;


	}

	cv::Mat Tracking::grab_image_monocular(const cv::Mat& im, const double& timestamp)
	{
		if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
			m_current_frame = Frame(im, timestamp, 4000, mK, mDistCoef);
		else
			m_current_frame = Frame(im, timestamp, 2000, mK, mDistCoef);

		if (mState == NO_IMAGES_YET) {
			mState = NOT_INITIALIZED;
		}
		mLastProcessedState = mState;

		if (mState == NOT_INITIALIZED)
		{
			if(frame_vector.size()==0) // Frame 0
				frame_vector.push_back(m_current_frame);
			else // Frame 1
			{
				auto macher = cv::FlannBasedMatcher::create();
				std::vector<std::vector<cv::DMatch>> matches;
				macher->knnMatch(m_current_frame.m_descriptors, frame_vector.back().m_descriptors, matches,2);
				const float ratio_thresh = 0.7f;
				std::vector<cv::DMatch> good_matches;
				for (size_t i = 0; i < matches.size(); i++) {
					if (matches[i][0].distance < ratio_thresh * matches[i][1].distance) {
						good_matches.push_back(matches[i][0]);
					}
				}

				std::vector<cv::Point2f> points1;
				std::vector<cv::Point2f> points2;

				cv::findFundamentalMat(points1, points2);
			}
		}

		
		return cv::Mat();
	}
}
