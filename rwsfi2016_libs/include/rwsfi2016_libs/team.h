#ifndef _TEAM_H_
#define _TEAM_H_

/* _________________________________
   |                                 |
   |           INCLUDES              |
   |_________________________________| */

//System includes
#include <iostream>
#include <vector>

//Boost includes
#include <boost/shared_ptr.hpp>

//ROS includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


/* _________________________________
   |                                 |
   |              CODE               |
   |_________________________________| */

using namespace std;
using namespace boost;
using namespace ros;

/**
 * @brief The namespace of this lib
 */
namespace rwsfi2016_libs
{

  /**
   * @brief Contains a list of all the players on a team
   */
  class Team
  {
    public: 
      /**
       * @brief Constructor
       * @param team_name the team name
       */
      Team(string team_name)
      {
        name = team_name; 

        vector<string> list_of_players;
        NodeHandle node;
        node.getParam(name, list_of_players);

        if (!list_of_players.size())
        {
          ROS_ERROR("Could not get %s team parameters. Make sure rosparams red, green and blue are set. For example:\nrosparam set red [\"red_player1\",\"red_player2\"]", name.c_str());
          shutdown();
        }

        for (size_t i=0; i < list_of_players.size(); ++i)
        {
          players.push_back(list_of_players[i]);
        }
      }

      /**
       * @brief Prints the name of the team and the names of all its players
       */
      void printTeamInfo(void)
      {
        cout << "Team " << name << " has the following players:" << endl;

        for (size_t i=0; i < players.size(); ++i)
          cout << players[i] << endl;
      }

      /**
       * @brief Checks if a player belongs to the team
       * @param player_name the name of the player to check
       * @return true or false, yes or no
       */
      bool playerBelongsToTeam(string player_name)
      {
        if(find(players.begin(), players.end(), player_name) != players.end())
          return true;
        else
          return false;
      }

      /**
       * @brief The team name
       */
      string name;

      /**
       * @brief A list of the team's players
       */
      vector<string> players;


  };//end of class Team
};//end of namespace rwsfi_libs

#endif
