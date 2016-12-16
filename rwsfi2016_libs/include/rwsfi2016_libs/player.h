#ifndef _PLAYER_H_
#define _PLAYER_H_

/* _________________________________
   |                                 |
   |           INCLUDES              |
   |_________________________________| */

//System includes
#include <iostream>

//ROS includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

/* _________________________________
   |                                 |
   |              CODE               |
   |_________________________________| */

#define DEFAULT_TIME 0.1 /*Default time to wait for a transform is 0.1 secs*/

using namespace std;
using namespace ros;

/**
 * @brief The namespace of this lib
 */
namespace rwsfi2016_libs
{

  /**
   * @brief Contains a description and methods for a game player
   */
  class Player
  {
    public:

      /* _________________________________
         |                                 |
         |  Constructor and other methods  |
         |_________________________________| */
      /**
       * @brief The constructor
       * @param player_name the name of the player
       */
      Player(string player_name) {this->name = player_name;}

      /**
       * @brief Set the name of the player's team
       * @param team_name a string with the team name
       */
      void setTeamName(string team_name)
      {
        if (team_name=="red" || team_name=="green" || team_name=="blue")
        {
          this->team = team_name;
        }
        else
        {
          cout << "cannot set team name to " << team_name << endl;
        }
      }

      /**
       * @brief Distance from this player to another player p
       * @param p an instance of the other player
       * @return distance in meters
       */
      double getDistanceToPLayer(string other_player, double time_to_wait=DEFAULT_TIME)
      {
        tf::StampedTransform t; //The transform object
        Time now = Time::now(); //get the time

        try //get the transformation between both players
        {
          listener.waitForTransform(name, other_player, now, Duration(2.0));
          listener.lookupTransform(name, other_player, now, t);
        }
        catch (tf::TransformException& ex){
          ROS_ERROR("%s",ex.what());
          Duration(0.1).sleep();
          return NAN; //return distance -1
        }

        return sqrt(t.getOrigin().x() *  t.getOrigin().x() +
                    t.getOrigin().y() *  t.getOrigin().y());
      }

      /**
       * @brief Angle between this player and another player
       * @param other_player the other player's name
       * @return angle in radians
       */
      double getAngleToPLayer(string other_player, double time_to_wait=DEFAULT_TIME)
      {
        tf::StampedTransform t; //The transform object
        Time now = Time::now(); //get the time

        try{
          listener.waitForTransform(name, other_player, now, Duration(time_to_wait));
          listener.lookupTransform(name, other_player, now, t);
        }
        catch (tf::TransformException& ex){
          ROS_ERROR("%s",ex.what());
          return NAN;
        }

        return atan2(t.getOrigin().y(), t.getOrigin().x());
      }


      /**
       * @brief Gets the pose of this player
       * @return the transform from /map to the player's local reference frame
       */
      tf::StampedTransform getPose(double time_to_wait=DEFAULT_TIME)
      {
        tf::StampedTransform t; //The transform object
        Time now = Time::now(); //get the time

        try{
          listener.waitForTransform("/map", name, now, Duration(time_to_wait));
          listener.lookupTransform("/map", name, now, t);
        }
        catch (tf::TransformException& ex){
          ROS_ERROR("%s",ex.what());
        }

        return t;
      }

      /* _________________________________
         |                                 |
         |          PROPPERTIES            |
         |_________________________________| */
      /**
       * @brief the name of the player
       */
      string name; 

      /**
       * @brief The name of the team
       */
      string team;

      /**
       * @brief the transform listener object
       */
      tf::TransformListener listener; //reads tfs from the ros system



  };//end of class Player
};//end of namespace rwsfi_libs


#endif
