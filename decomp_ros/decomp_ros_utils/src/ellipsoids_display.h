#include <boost/circular_buffer.hpp>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include <rviz/load_resource.h>

#include <rviz/message_filter_display.h>
#include "ellipsoids_visual.h"

namespace decomp_rviz_plugins {

  class EllipsoidsVisual;

  class EllipsoidsDisplay
    : public rviz::MessageFilterDisplay<decomp_ros_msgs::Ellipsoids> {
      Q_OBJECT
      public:
        EllipsoidsDisplay();
        virtual ~EllipsoidsDisplay();

      protected:
        virtual void onInitialize();

        virtual void reset();

        private Q_SLOTS:
          void updateColorAndAlpha();
        void updateHistoryLength();

      private:
        void processMessage(const decomp_ros_msgs::Ellipsoids::ConstPtr &msg);

        boost::circular_buffer<boost::shared_ptr<EllipsoidsVisual>> visuals_;

        rviz::ColorProperty *color_property_;
        rviz::FloatProperty *alpha_property_;
        rviz::IntProperty *history_length_property_;
    };

}
