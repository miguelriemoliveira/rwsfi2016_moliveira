
#include <ros/ros.h>
//#include <tf/transform_broadcaster.h>

#include <rwsfi2016_libs/team_info.h>
//#include <rws2016_msgs/GameMove.h>

using namespace std;


/**
 * @brief MyPlayer extends class Player, i.e., there are additional things I can do with MyPlayer and not with any Player, e.g., to order a movement.
 */
class MyPlayer: public Player
{
  public: 


    boost::shared_ptr<ros::Subscriber> _sub; 

    ~MyPlayer()   
    {
      tf::Transform t;
      t.setOrigin( tf::Vector3(15, 15, 0.0) );
      tf::Quaternion q; q.setRPY(0, 0, 0);
      t.setRotation(q);
      br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "/map", name));
      br.sendTransform(tf::StampedTransform(t, ros::Time::now() + ros::Duration(2), "/map", name));
    }

    /**
     * @brief Constructor
     *
     * @param name player name
     * @param team team name
     */
    MyPlayer(string name, string team): Player(name)
  {
    setTeamName(team);
    ros::NodeHandle node;

    //Initialize teams
    vector<string> myTeam_names, myHunters_names, myPreys_names;
    string myTeamId, myHuntersId, myPreysId;

    if (!team_info(node, myTeam_names, myHunters_names, myPreys_names, myTeamId, myHuntersId, myPreysId))
      ROS_ERROR("Something went wrong reading teams");

    my_team = (boost::shared_ptr<Team>) new Team(myTeamId, myTeam_names);
    hunter_team = (boost::shared_ptr<Team>) new Team(myHuntersId, myHunters_names);
    prey_team = (boost::shared_ptr<Team>) new Team(myPreysId, myPreys_names);

    my_team->printTeamInfo();
    hunter_team->printTeamInfo();
    prey_team->printTeamInfo();

    //Initialize position according to team
    ros::Duration(0.5).sleep(); //sleep to make sure the time is correct
    tf::Transform t;
    //srand((unsigned)time(NULL)); // To start the player in a random location
    struct timeval t1;      
    gettimeofday(&t1, NULL);
    srand(t1.tv_usec);
    double X=((((double)rand()/(double)RAND_MAX) ) * 2 -1) * 5 ;
    double Y=((((double)rand()/(double)RAND_MAX) ) * 2 -1) * 5 ;
    t.setOrigin( tf::Vector3(X, Y, 0.0) );
    tf::Quaternion q; q.setRPY(0, 0, 0);
    t.setRotation(q);
    br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "/map", name));

    //initialize the subscriber
    _sub = (boost::shared_ptr<ros::Subscriber>) new ros::Subscriber;
    *_sub = node.subscribe("/game_move", 1, &MyPlayer::moveCallback, this);

  }


    void getNameOfClosestPrey(string& name, double& distance)
    {
      double prey_dist = getDistance(*prey_team->players[0]);
      string prey_name = prey_team->players[0]->name;

      for (size_t i = 1; i < prey_team->players.size(); ++i)
      {
        double d = getDistance(*prey_team->players[i]);

        if (d < prey_dist) //A new minimum
        {
          prey_dist = d;
          prey_name = prey_team->players[i]->name;
        }
      }

      name = prey_name;
      distance = prey_dist;
    }

    void getNameOfClosestHunter(string& name, double& distance)
    {
      double hunter_dist = getDistance(*hunter_team->players[0]);
      string hunter_name = hunter_team->players[0]->name;

      for (size_t i = 1; i < hunter_team->players.size(); ++i)
      {
        double d = getDistance(*hunter_team->players[i]);

        if (d < hunter_dist) //A new minimum
        {
          hunter_dist = d;
          hunter_name = hunter_team->players[i]->name;
        }
      }

      name = hunter_name;
      distance = hunter_dist;
    }



    /**
     * @brief called whenever a /game_move msg is received
     *
     * @param msg the msg with the animal values
     */
    void moveCallback(const rws2016_msgs::GameMove& msg)
    {
      ROS_INFO("player %s received game_move msg", name.c_str()); 

      //I will encode a very simple hunting behaviour:
      //
      //1. Get names of closest prey and hunter 
      //2. Get angle to closest prey
      //3. Compute maximum displacement
      //4. Move maximum displacement towards angle to prey (limited by min, max)

      //Step 1
      string closest_prey; double dist_closest_prey;
      getNameOfClosestPrey(closest_prey, dist_closest_prey);
      ROS_INFO("Closest prey is %s", closest_prey.c_str());

      //string closest_hunter; double dist_closest_hunter;
      //getNameOfClosestHunter(closest_hunter, dist_closest_hunter);
      //ROS_INFO("Closest hunter is %s", closest_hunter.c_str());



      //Step 2
      double angle = 0;
      //if (dist_closest_hunter <  dist_closest_prey)
      //{
      //angle = getAngle(closest_hunter) + M_PI;
      //}
      //else
      //{
      //angle = getAngle(closest_prey);
      //}

      angle = getAngleFromPrey(closest_prey);

      //Step 3
      double displacement = msg.cat; //I am a cat, others may choose another animal

      //Step 4
      move(displacement, angle);

    }

};



/**
 * @brief The main function
 *
 * @param argc number of command line arguments
 * @param argv values of command line arguments
 *
 * @return result
 */
int main(int argc, char** argv)
{
  //initialize ROS stuff
  ros::init(argc, argv, "moliveira");
  ros::NodeHandle node;

  //Creating an instance of class MyPlayer
  rws2016_moliveira::MyPlayer my_player("moliveira", "red");

  //Infinite loop
  ros::spin();
}
