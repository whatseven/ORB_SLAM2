#include "main_system.h"
#include "common_util.h"

#include <thread>
#include <iomanip>

#include <pangolin/pangolin.h>


using namespace std;
namespace MONOCULAR_ORB_SLAM2 {
    System::System(const string& strSettingsFile,
        const bool bUseViewer) {
        //Check settings file
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
        if (!fsSettings.isOpened()) {
            cerr << "Failed to open settings file at: " << strSettingsFile << endl;
            exit(-1);
        }

        m_tracker = new Tracking(strSettingsFile);
    }

    cv::Mat System::track_monocular(const cv::Mat& im, const double& timestamp) {
        m_tracker->grab_image_monocular(im, timestamp);
    	
        return cv::Mat();
    }

} //namespace ORB_SLAM
