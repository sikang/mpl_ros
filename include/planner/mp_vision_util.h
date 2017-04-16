#include <planner/astar.h>
#include <planner/env_vision.h>
#include <primitive/trajectory.h>

class MPVisionUtil
{
  public:
    MPVisionUtil(bool verbose);
    bool plan(const Waypoint &start, const Waypoint &goal);
    Trajectory getTraj();
    std::vector<Waypoint> getPath() { return path_; }
    std::vector<Primitive> getPrimitives() { return ENV_->primitives(); }
    cv::Mat getImage() { return ENV_->image(getTraj()); }
    vec_Vec3f getPs() { return ENV_->ps(); }
    vec_Vec3f getCloud() { return ENV_->cloud(); }
    void setVmax(decimal_t v);
    void setAmax(decimal_t a);
    void setDt(decimal_t a);
    void setEpsilon(decimal_t eps);
    void addImage(const cv::Mat& img, const Aff3f& TF, const CameraInfo& info);


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
 protected:
    bool _planner_verbose;
    std::unique_ptr<mrsl::env_vision> ENV_;
    std::vector<Waypoint> path_;
    decimal_t epsilon_ = 1.0;
};


