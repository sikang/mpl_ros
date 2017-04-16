#include <collision_checking/vision_util.h>

VisionUtil::VisionUtil() {}

VisionUtil::VisionUtil(const cv::Mat& img, const CameraInfo& info) : 
  img_(img), info_(info) {
    img_inflated_ = cv::Mat(info_.height, info_.width, CV_32FC1, cv::Scalar(0));
  }

void VisionUtil::setBound(decimal_t u, decimal_t v) {
  upper_bound_ = u;
  lower_bound_ = v;
}

Vec3f VisionUtil::intToFloat(const Vec2i& pn, double d) {
  return Vec3f((pn(0) - info_.center_x)* d * info_.constant_x,
      (pn(1) - info_.center_y) * d * info_.constant_y, d);
}

Vec2i VisionUtil::floatToInt(const Vec3f& pt) {
  //float d = depth_image_proc::DepthTraits<float>::fromMeters(pt(2));
  return Vec2i(pt(0) / info_.constant_x / pt(2) + info_.center_x,
      pt(1) / info_.constant_y / pt(2) + info_.center_y);
}

bool VisionUtil::isOutside(const Vec3f& pt) {
  if(pt(2) > upper_bound_ || pt(2) < lower_bound_)
    return true;

  Vec2i pn = floatToInt(pt);
  return isOutside(pn);
 }

bool VisionUtil::isFree(const Vec3f& pt) {
  if(isOutside(pt))
    return false;

  Vec2i pn = floatToInt(pt);
  return !isOccupied(pn, pt(2));
}

bool VisionUtil::isOutside(const Vec2i& pn) {
  return pn(0) >= info_.width || pn(0) < 0 ||
    pn(1) >= info_.height || pn(1) < 0;
}

bool VisionUtil::isOccupied(const Vec2i& pn, decimal_t d) {
  float dmax = img_.at<float>(pn(1), pn(0));
  float dmax_inf = img_inflated_.at<float>(pn(1), pn(0));
  if((!std::isnan(dmax) && d > dmax) ||
      (dmax_inf > 0 && std::fabs(d - dmax_inf) < 0.1))
    return true;
  else
    return false;
}

bool VisionUtil::isVeryFree(const Vec3f& pt) {
  decimal_t r = 0.1;
  decimal_t h = 0.1;

  int rx = r / pt(2) / info_.constant_x;
  int ry = h / pt(2) / info_.constant_y;

  const Vec2i pn = floatToInt(pt);
  if(isOutside(pt))
    return false;

  for(int dx = -rx; dx <= rx; dx ++) {
    for(int dy = -ry; dy <= ry; dy ++) {
      Vec2i new_pn = pn + Vec2i(dx, dy);
      if(!isOutside(new_pn) && 
          isOccupied(new_pn, pt(2) + r))
        return false;
    }
  }

  return true;
}

bool VisionUtil::isFree(const Primitive& pr, const Aff3f& TF) {
  std::vector<Waypoint> pts = pr.sample(10);
  for(const auto& pt: pts){
    Vec3f pt_b = TF * pt.pos;
    Vec2i pn = floatToInt(pt_b);
    if(!isOutside(pn) && isOccupied(pn, pt_b(2)))
      return false;
  }

  return true;
}

vec_Vec3f VisionUtil::cloud(const cv::Mat& img) {
  vec_Vec3f pts;
  for (int v = 0; v < info_.height; v ++) {
    for (int u = 0; u < info_.width; u ++) {
      float d = img.at<float>(v, u);
      // Check for invalid measurements
      if (!std::isnan(d) && d >= lower_bound_ && d <= upper_bound_) {
	pts.push_back(intToFloat(Vec2i(u, v), d));
      }
    }
  }

  return pts;
}


cv::Mat VisionUtil::img() {
  return img_;
}


cv::Mat VisionUtil::img_inflated() {
  return img_inflated_;
}

cv::Mat VisionUtil::drawing(const cv::Mat& img, const Trajectory& traj, const Aff3f& TF) {
  std::vector<Waypoint> pts = traj.sample(100);
  cv::Mat img_drawing;
  cv::normalize(img, img_drawing, 1, 0, cv::NORM_MINMAX);
  for(int i = 0; i < pts.size()-1; i++){
    Vec3f pt1_b = TF * pts[i].pos;
    Vec3f pt2_b = TF * pts[i+1].pos;

    if(pt1_b(2) > 0.1 && pt2_b(2) > 0.1) {
      Vec2i pn1 = floatToInt(pt1_b);
      Vec2i pn2 = floatToInt(pt2_b);
      //std::cout << pn1.transpose() << "raw: " << pt1_b.transpose() << std::endl;
      if(!isOutside(pn1) && !isOutside(pn2))
        cv::line(img_drawing, cv::Point(pn1(0), pn1(1)), cv::Point(pn2(0), pn2(1)), cv::Scalar(1), 1);
      //cv::circle(img_drawing, cv::Point(pn1(0), pn1(1)), 1, cv::Scalar(1), 1);
    }
  }

  return img_drawing;
}

void VisionUtil::inflate(decimal_t r, decimal_t h) {
  cv::Mat img_inf(info_.height, info_.width, CV_32FC1, cv::Scalar(0));
  for (int v = 0; v < info_.height; v ++) {
    for (int u = 0; u < info_.width; u ++) {
      if(std::isnan(img_.at<float>(v, u)) || img_.at<float>(v, u) < lower_bound_ || img_.at<float>(v, u) > upper_bound_)
        continue;
      float d = img_.at<float>(v, u);
      int rx = r / d / info_.constant_x;
      int ry = h / d / info_.constant_y;
      Vec3f pt  = intToFloat(Vec2i(u, v), d);
      for(int dx = -rx; dx <= rx; dx ++) {
        for(int dy = -ry; dy <= ry; dy ++) {
          Vec2i pn(u + dx, v+ dy);
          if(!isOutside(pn)) {
            if(img_inf.at<float>(pn(1), pn(0)) == 0 ||
                img_inf.at<float>(pn(1), pn(0)) > d)
              img_inf.at<float>(pn(1), pn(0)) = d;
          }
        }
      }
    }
  }

  img_inflated_ = img_inf;
}
