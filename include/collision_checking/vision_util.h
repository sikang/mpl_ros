/**
 * @file vision_util.h
 * @brief Vision util class
 */
#ifndef VISION_UTIL_H
#define VISION_UTIL_H
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <primitive/primitive.h>
#include <primitive/trajectory.h>

/**
 * @brief Camera info used to get depth from disparity
 */
struct CameraInfo {
  decimal_t center_x;
  decimal_t center_y;

  decimal_t constant_x;
  decimal_t constant_y;

  int width;
  int height;
};

/**
 * @brief Collision checking inside the image space
 */
class VisionUtil {
  public:
    ///Simple constructor
    VisionUtil();
    ///Constructor with cv image and camera info
    VisionUtil(const cv::Mat& img, const CameraInfo& info);
    ///Bounding box in depth
    void setBound(decimal_t u, decimal_t l);
    ///Check if the point is outside of the image after projection
    bool isOutside(const Vec3f& pt);
    ///Check if the point is in free space without occlusion
    bool isVeryFree(const Vec3f& pt);
    ///Check if the point is not occupied
    bool isFree(const Vec3f& pt);
    ///Check if the primitive is free
    bool isFree(const Primitive& pr, const Aff3f& TF);
    ///Get the image
    cv::Mat img();
    ///Get the inflated image
    cv::Mat img_inflated();
    ///Project the trajectory into image space
    cv::Mat drawing(const cv::Mat& img, const Trajectory& traj, const Aff3f& TF);
    ///Get the point cloud from disparity image
    vec_Vec3f cloud(const cv::Mat& img);

    ///Check if the image point is outside
    bool isOutside(const Vec2i& pn);
    ///Check if the image point is on obstacle
    bool isOccupied(const Vec2i& pn, decimal_t dt);
    ///Convert image point to 3D point
    Vec3f intToFloat(const Vec2i& pn, double d);
    ///Convert 3D point into image point
    Vec2i floatToInt(const Vec3f& pt);

    ///Inflate obstacle in image space
    void inflate(decimal_t r, decimal_t h);
  private:
    cv::Mat img_;
    cv::Mat img_inflated_;
    CameraInfo info_;
    decimal_t upper_bound_ = 4, lower_bound_ = -0.2;
};

#endif
