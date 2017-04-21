#include <planner/env_vision.h>
#include <planner/mp_base_util.h>

class MPVisionUtil : public MPBaseUtil
{
  public:
    MPVisionUtil(bool verbose);
    bool plan(const Waypoint &start, const Waypoint &goal);
    cv::Mat getImage();
    void addImage(const cv::Mat& img, const Aff3f& TF, const CameraInfo& info);
    std::unique_ptr<mrsl::env_vision> ENV_;
};


