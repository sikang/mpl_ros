#ifndef VISION_UTIL_H
#define VISION_UTIL_H
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <primitive/primitive.h>
#include <primitive/trajectory.h>

struct CameraInfo {
  decimal_t center_x;
  decimal_t center_y;

  decimal_t constant_x;
  decimal_t constant_y;

  int width;
  int height;
};

class VisionUtil {
  public:
    VisionUtil();
    ~VisionUtil() = default;
    VisionUtil(const cv::Mat& img, const CameraInfo& info);
    void setBound(decimal_t u, decimal_t l);
    bool isOutside(const Vec3f& pt);
    bool isVeryFree(const Vec3f& pt);
    bool isFree(const Vec3f& pt);
    bool isFree(const Primitive& pr, const Aff3f& TF);
    cv::Mat img();
    cv::Mat img_inflated();
    cv::Mat drawing(const cv::Mat& img, const Trajectory& traj, const Aff3f& TF);
    vec_Vec3f cloud(const cv::Mat& img);

    bool isOutside(const Vec2i& pn);
    bool isOccupied(const Vec2i& pn, decimal_t dt);
    Vec3f intToFloat(const Vec2i& pn, double d);
    Vec2i floatToInt(const Vec3f& pt);

    void inflate(decimal_t r, decimal_t h);
  private:
    cv::Mat img_;
    cv::Mat img_inflated_;
    CameraInfo info_;
    decimal_t upper_bound_ = 4, lower_bound_ = -0.2;
};

#endif
