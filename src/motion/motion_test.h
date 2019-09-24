/**
 * Main Motion Controlling Class
 * Stores Arming State & basic subscribers
 *
 */

#pragma once

#include <iostream>
#include <ros/ros.h>

// We inlcude Stamped messages because we want to Track the time 
// when messages were last updated
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

class Motion {
  
  public:

    Motion(bool sitl = false);


    bool getArmState();

    void updateArmState();

    void setPose(geometry_msgs::Pose);

    void setVelocity(geometry_msgs::TwistStamped);

    void setAccel(geometry_msgs::TwistStamped);

    geometry_msgs::TwistStamped getVelocity();

    geometry_msgs::TwistStamped getAccel();

    geometry_msgs::Pose getPose();

    // call this function every updateTime seconds
    // updates all velocity, pose, & acceleration periods
    void Update();

  private:

    static bool armState_;

    geometry_msgs::TwistStamped velocity_;
    geometry_msgs::TwistStamped accel_;
    geometry_msgs::Pose pose_;
    
    ros::Timer curr_time_;

    ros::Timer updateTime_;

    ros::NodeHandle nh_;
}
