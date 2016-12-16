#ifndef _PLAYER_H_
#define _PLAYER_H_

/* _________________________________
   |                                 |
   |           INCLUDES              |
   |_________________________________| */

//System includes
#include <iostream>
#include <algorithm>

//ROS includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

//my libraries includes
#include <rwsfi2016_libs/team.h>

/* _________________________________
   |                                 |
   |              CODE               |
   |_________________________________| */

#define DEFAULT_TIME 0.1 /*Default time to wait for a transform is 0.1 secs*/

using namespace std;
using namespace ros;
using namespace tf;
using namespace boost;

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
      Player(string player_name)
      {
        name = player_name; //set the name
        ROS_INFO("my player name is %s", name.c_str());

        //Given the list of players of each "/teamRed" "/teamGreen" and "teamBlue", decide which is my_team, hunters_team and preys_team
        vector<string> red_players, green_players, blue_players;

        if(!node.getParam("/teamRed", red_players) ||
            !node.getParam("/teamGreen", green_players) ||
            !node.getParam("/teamBlue", blue_players) )
        {
          if(find(red_players.begin(), red_players.end(), name) != red_players.end())
          {
            my_team = (shared_ptr<Team>) new Team("red", red_players);
            preys_team = (shared_ptr<Team>) new Team("green", green_players);
            hunters_team = (shared_ptr<Team>) new Team("blue", blue_players);
          }
          else if(find(green_players.begin(), green_players.end(), name) != green_players.end())
          {
            my_team = (shared_ptr<Team>) new Team("green", green_players);
            preys_team = (shared_ptr<Team>) new Team("blue", blue_players);
            hunters_team = (shared_ptr<Team>) new Team("red", red_players);
          }
          else if(find(blue_players.begin(), blue_players.end(), name) != blue_players.end())
          {
            my_team = (shared_ptr<Team>) new Team("blue", blue_players);
            preys_team = (shared_ptr<Team>) new Team("red", red_players);
            hunters_team = (shared_ptr<Team>) new Team("green", green_players);
          }
        }

        my_team->printTeamInfo();
        hunters_team->printTeamInfo();
        preys_team->printTeamInfo();


      }


      /**
       * @brief Distance from this player to another player p
       * @param p an instance of the other player
       * @return distance in meters
       */
      double getDistanceToPLayer(string other_player, double time_to_wait=DEFAULT_TIME)
      {
        StampedTransform t; //The transform object
        Time now = Time::now(); //get the time

        try //get the transformation between both players
        {
          listener.waitForTransform(name, other_player, now, Duration(2.0));
          listener.lookupTransform(name, other_player, now, t);
        }
        catch (TransformException& ex){
          ROS_ERROR("%s",ex.what());
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
        StampedTransform t; //The transform object
        Time now = Time::now(); //get the time

        try{
          listener.waitForTransform(name, other_player, now, Duration(time_to_wait));
          listener.lookupTransform(name, other_player, now, t);
        }
        catch (TransformException& ex){
          ROS_ERROR("%s",ex.what());
          return NAN;
        }

        return atan2(t.getOrigin().y(), t.getOrigin().x());
      }


      /**
       * @brief Gets the pose of this player
       * @return the transform from /map to the player's local reference frame
       */
      StampedTransform getPose(double time_to_wait=DEFAULT_TIME)
      {
        StampedTransform t; //The transform object
        Time now = Time::now(); //get the time

        try{
          listener.waitForTransform("/map", name, now, Duration(time_to_wait));
          listener.lookupTransform("/map", name, now, t);
        }
        catch (TransformException& ex){
          ROS_ERROR("%s",ex.what());
        }

        return t;
      }

      /**
       * @brief Moves the player by publishing a transform
       * @param displacement the linear movement of the player, bounded by [0, 1]
       * @param turn_angle the turn angle of the player, bounded by [-M_PI/30, M_PI/30]
       */
      void move(double displacement, double turn_angle)
      {
        //Put arguments withing authorized boundaries
        displacement = (displacement > 1 ? 1 : displacement);
        displacement = (displacement < 0 ? 0 : displacement);

        double max_t =  (M_PI/30);
        if (turn_angle > max_t) turn_angle = max_t;
        else if (turn_angle < -max_t) turn_angle = -max_t;

        //Compute the new reference frame
        Transform t_mov;
        t_mov.setOrigin( Vector3(displacement , 0.0, 0.0) );
        Quaternion q;
        q.setRPY(0, 0, turn_angle);
        t_mov.setRotation(q);

        Transform t = getPose();
        t = t  * t_mov;

        //Send the new transform to ROS
        broadcaster.sendTransform(StampedTransform(t, Time::now(), "/map", name));
      }

      /* _________________________________
         |                                 |
         |          PROPPERTIES            |
         |_________________________________| */
      /**
       * @brief the name of the player and team
       */
      string name;

      /**
       * @brief the transform listener object
       */
      TransformListener listener; //reads tfs from the ros system

      /**
       * @brief The transform publisher object
       */
      TransformBroadcaster broadcaster;

      /**
       * @brief The teams
       */
      shared_ptr<Team> my_team, hunters_team, preys_team;


      shared_ptr<Subscriber> _sub; 

      NodeHandle node;



  };//end of class Player
};//end of namespace rwsfi_libs


#endif
