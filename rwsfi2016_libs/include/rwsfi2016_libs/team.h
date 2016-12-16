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
       * @param player_names a list with the name of all the players on the team
       */
      Team(string team_name, vector<string>& player_names)
      {
        name = team_name; 

        for (size_t i=0; i < player_names.size(); ++i)
        {
          players.push_back(player_names[i]);
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
