/**
 * @file vision_queue_util.h
 * @brief Vision queue util class
 */
#ifndef VISION_QUEUE_UTIL_H
#define VISION_QUEUE_UTIL_H
#include <memory>
#include <deque>
#include <motion_primitive_library/data_utils.h>
#include <collision_checking/vision_util.h>

/**
 * @brief Stack of VisionUtil class
 */
class VisionQueueUtil {
  public:
    ///Simple constructor
    VisionQueueUtil();
    ///Add an image object in queue
    void addImage(const cv::Mat& img, const Aff3f& TF, const CameraInfo& info);
    ///Check if the point is outside all images in buffer
    bool isOutside(const Vec3f& pt);
    ///Check if the point is in free space in all images in buffer
    bool isFree(const Vec3f& pt);
    ///Check if the primitive is in free space in all images in buffer
    bool isFree(const Primitive& pr);
    ///Get the k-th image with projected trajectory 
    cv::Mat img(int k, const Trajectory& traj);
    ///Get point cloud
    vec_Vec3f cloud();
    ///Get size of images in buffer
    int size();

  private:
    ///Queue object
    std::deque<std::shared_ptr<VisionUtil>> vision_utils_;
    ///Corresponding TF from world to image frame
    std::deque<Aff3f> TFs_;
};

#endif
