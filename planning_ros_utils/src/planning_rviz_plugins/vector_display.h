#include <boost/circular_buffer.hpp>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/frame_manager.h>

#include <rviz/load_resource.h>

#include <planning_ros_msgs/Arrows.h>
#include <rviz/message_filter_display.h>

#include "vector_visual.h"
#include "path_visual.h"

namespace mav_rviz_plugins {
  class VectorDisplay
    : public rviz::MessageFilterDisplay<planning_ros_msgs::Arrows> {
      Q_OBJECT
      public:
        VectorDisplay();
        virtual ~VectorDisplay();

      protected:
        virtual void onInitialize();

        virtual void reset();

        private Q_SLOTS:
          void updateLineColorAndAlpha();
        void updateVsColorAndAlpha();
        void updateLineScale();
        void updateVsScale();
        void updateState();

      private:
        void processMessage(const planning_ros_msgs::Arrows::ConstPtr &msg);
        void visualizeMessage(int state);
        void visualizePath();
        void visualizeVs();

        boost::circular_buffer<boost::shared_ptr<PathVisual>> visuals_path_;
        boost::circular_buffer<boost::shared_ptr<VectorVisual>> visuals_vs_;

        rviz::ColorProperty *line_color_property_;
        rviz::ColorProperty *vs_color_property_;
        rviz::FloatProperty *line_scale_property_;
        rviz::FloatProperty *vs_scale_property_;
        rviz::EnumProperty *state_property_;

        Ogre::Vector3 position_;
        Ogre::Quaternion orientation_;

        vec_Vec3f path_;
        vec_E<std::pair<Vec3f, Vec3f>> vs_;
    };

}
