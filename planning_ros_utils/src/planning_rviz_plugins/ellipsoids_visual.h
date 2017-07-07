#ifndef ELLIPSOIDS_VISUAL_H
#define ELLIPSOIDS_VISUAL_H

#include <planning_ros_msgs/Ellipsoids.h>
#include <motion_primitive_library/data_type.h>
#include <Eigen/Eigenvalues>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/shape.h>

namespace mav_rviz_plugins {
  class EllipsoidsVisual {
    public:
      EllipsoidsVisual(Ogre::SceneManager *scene_manager,
                       Ogre::SceneNode *parent_node);

      virtual ~EllipsoidsVisual();

      void setMessage(const planning_ros_msgs::Ellipsoids::ConstPtr &msg);

      void setFramePosition(const Ogre::Vector3 &position);
      void setFrameOrientation(const Ogre::Quaternion &orientation);

      void setColor(float r, float g, float b, float a);

    private:
      std::vector<std::unique_ptr<rviz::Shape>> objs_;

      Ogre::SceneNode *frame_node_;

      Ogre::SceneManager *scene_manager_;
  };
}
#endif
