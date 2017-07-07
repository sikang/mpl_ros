#include <boost/circular_buffer.hpp>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/frame_manager.h>

#include <rviz/load_resource.h>

#include <planning_ros_msgs/Trajectories.h>
#include <rviz/message_filter_display.h>

#include "trajectory_visual.h"

namespace mav_rviz_plugins {
  class TrajectoriesDisplay
    : public rviz::MessageFilterDisplay<planning_ros_msgs::Trajectories> {
      Q_OBJECT
      public:
        TrajectoriesDisplay();
        virtual ~TrajectoriesDisplay();

      protected:
        virtual void onInitialize();

        virtual void reset();

        private Q_SLOTS:
          void updatePosColorAndAlpha();
        void updateVelColorAndAlpha();
        void updateAccColorAndAlpha();
        void updatePosScale();
        void updateVelScale();
        void updateAccScale();
        void updateVelVis();
        void updateAccVis();
        void updateHistoryLength();
        void updateNum();

      private:
        void processMessage(const planning_ros_msgs::Trajectories::ConstPtr &msg);
        void visualizeMessage();

        boost::circular_buffer<boost::shared_ptr<TrajectoryVisual>> visuals_;

        rviz::ColorProperty *pos_color_property_;
        rviz::ColorProperty *vel_color_property_;
        rviz::ColorProperty *acc_color_property_;
        rviz::FloatProperty *pos_scale_property_;
        rviz::FloatProperty *vel_scale_property_;
        rviz::FloatProperty *acc_scale_property_;
        rviz::BoolProperty *vel_vis_property_;
        rviz::BoolProperty *acc_vis_property_;
        rviz::IntProperty *history_length_property_;
        rviz::IntProperty *num_property_;

        Ogre::Vector3 position_;
        Ogre::Quaternion orientation_;

        planning_ros_msgs::Trajectories trajectories_;
    };

}
