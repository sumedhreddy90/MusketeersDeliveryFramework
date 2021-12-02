#ifndef ROBOTDISPLAYGROUP_H
#define ROBOTDISPLAYGROUP_H

#include "rviz/display_group.h"
#include <string>
#include "rviz/visualization_manager.h"
#include "rviz/display.h"

class MuskyDisplayGroup : public rviz::DisplayGroup
{
public:
    MuskyDisplayGroup(std::string, rviz::VisualizationManager*);

private:
    std::string robotId_;
    
    //Pertaining to Robot Model display
    std::string modelParameter_;
    rviz::Display* robotModel_;
    
    //Pertaining to Robot Footprint display
    std::string footprintTopic_;
    rviz::Display* robotFootprint_;
    
    //Pertaining to Robot base_link Frame axes display
    std::string baselinkFrame_;
    rviz::Display* baselinkAxes_;

    //Pertaining to Robot odom_link Frame axes display
    std::string odomFrame_;
    rviz::Display* odomAxes_;

    //Pertaining to Lase Scan display
    std::string scanTopic_;
    rviz::Display* laserScan_;

    //Pertaining to AMCL Robot Pose Array display
    std::string poseTopic_;
    rviz::Display* poseArray_;

    //Pertaining to Goal Pose display
    std::string goalTopic_;
    rviz::Display* goalPose_;
    
    //Pertaining to Goal path plan display
    std::string globalPathTopic_;
    rviz::Display* globalPath_;

    //Pertaining to local path plan display
    std::string localPathTopic_;
    rviz::Display* localPath_;

    //Pertaining to global costmap display
    std::string globalCostmapTopic_;
    rviz::Display* globalCostmap_;

    //Pertaining to local costmap display
    std::string localCostmapTopic_;
    rviz::Display* localCostmap_;

    //Pertaining to Interactive Twist command marker display
    std::string interactiveTwistMarkerTopic_;
    rviz::Display* interactiveTwistMarker_;
};


#endif // ROBOTDISPLAYGROUP_H