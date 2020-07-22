#ifndef FRAME_H
#define FRAME_H

#include <vector>

#include <opencv2/opencv.hpp>

namespace MONOCULAR_ORB_SLAM2
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class Frame
{
public:
    Frame();
    Frame(const cv::Mat &imGray, const double &timeStamp,int feature_num, cv::Mat &K, cv::Mat &distCoef);
    void extract_orb(const cv::Mat& v_img, int feature_num);

public:
    double mTimeStamp;
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    int num_keypoints;
    std::vector<cv::KeyPoint> m_keypoints;
    cv::Mat m_descriptors;


private:

};

}
#endif // FRAME_H
