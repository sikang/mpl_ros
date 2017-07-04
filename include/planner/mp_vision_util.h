/**
 * @file mp_vision_util.h
 * @brief Motion primitive util class in image space
 */
#include <planner/env_vision.h>
#include <planner/mp_base_util.h>

/**
 * @brief Motion primitive plannner in image space
 */
class MPVisionUtil : public MPBaseUtil
{
  public:
    /**
     * @brief Simple constructor
     * @param verbose enable print out
     */
    MPVisionUtil(bool verbose);
    ///Planning from start to goal
    bool plan(const Waypoint &start, const Waypoint &goal);
    ///Get image for debugging
    cv::Mat getImage();
    ///Add an image object in buffer
    void addImage(const cv::Mat& img, const Aff3f& TF, const CameraInfo& info);
    ///Env object
    std::unique_ptr<MPL::env_vision> ENV_;
};


