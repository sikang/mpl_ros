#include <collision_checking/vision_queue_util.h>

VisionQueueUtil::VisionQueueUtil() {}

void VisionQueueUtil::addImage(const cv::Mat& image, const Aff3f& TF, const CameraInfo& info) {
  std::shared_ptr<VisionUtil> vision_util(new VisionUtil(image, info));
  //vision_util->inflate(0.3, 0.2);
  vision_utils_.push_back(vision_util);
  while(vision_utils_.size() > 10)
    vision_utils_.pop_front();

  TFs_.push_back(TF);
  while(TFs_.size() > 10)
    TFs_.pop_front();
}


bool VisionQueueUtil::isOutside(const Vec3f& pt) {
  for(int i = 0; i < (int) vision_utils_.size(); i++) {
    if(!vision_utils_[i]->isOutside(TFs_[i] * pt))
      return false;
  }

  return true;
}

bool VisionQueueUtil::isFree(const Vec3f& pt) {
  for(int i = 0; i < (int) vision_utils_.size(); i++) {
    if(vision_utils_[i]->isFree(TFs_[i] * pt))
      return true;
  }

  return false;
}

bool VisionQueueUtil::isFree(const Primitive& pr) {
  std::vector<Waypoint> pts = pr.sample(10);
  for(const auto& pt: pts) {
    if(!isOutside(pt.pos)) {
      bool valid = false;
      for(int i = 0; i < (int) vision_utils_.size(); i++) {
        Vec3f pt_b = TFs_[i] * pt.pos;
        if(vision_utils_[i]->isVeryFree( pt_b)) {
          valid = true;
          break;
        }
      }
      if(!valid)
        return false;
    }
  }

  return true;
}

cv::Mat VisionQueueUtil::img(int k, const Trajectory& traj) {
  return vision_utils_[k]->drawing(vision_utils_[k]->img(), traj, TFs_[k]);
}

vec_Vec3f VisionQueueUtil::cloud() {
  vec_Vec3f clouds;
  for(int i = 0; i < (int) vision_utils_.size(); i++) {
    vec_Vec3f cloud = transform_vec3(vision_utils_[i]->cloud(vision_utils_[i]->img()), TFs_[i].inverse());
    clouds.insert(clouds.end(), cloud.begin(), cloud.end());
  }
  return clouds;
}

int VisionQueueUtil::size() {
  return vision_utils_.size();
}
