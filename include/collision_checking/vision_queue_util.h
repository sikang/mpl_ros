#ifndef VISION_QUEUE_UTIL_H
#define VISION_QUEUE_UTIL_H
#include <memory>
#include <deque>
#include <basic_type/data_utils.h>
#include <collision_checking/vision_util.h>

class VisionQueueUtil {
  public:
    VisionQueueUtil();
    void addImage(const cv::Mat& img, const Aff3f& TF, const CameraInfo& info);
    bool isOutside(const Vec3f& pt);
    bool isFree(const Vec3f& pt);
    bool isFree(const Primitive& pr);
    cv::Mat img(int k, const Trajectory& traj);
    vec_Vec3f cloud();
    int size();

  private:
    std::deque<std::shared_ptr<VisionUtil>> vision_utils_;
    std::deque<Aff3f> TFs_;
};

#endif
