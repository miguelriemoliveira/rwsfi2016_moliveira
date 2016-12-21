/* _________________________________
   |                                 |
   |           INCLUDES              |
   |_________________________________| */
#include <ros/ros.h>
#include <rwsfi2016_libs/player.h>

/* _________________________________
   |                                 |
   |              CODE               |
   |_________________________________| */
using namespace std;
using namespace ros;


/**
 * @brief MyPlayer extends class Player, i.e., there are additional things I can do with MyPlayer and not with any Player, e.g., to order a movement.
 */
class MyPlayer: public rwsfi2016_libs::Player
{
    public: 

        /**
         * @brief Constructor, nothing to be done here
         * @param name player name
         * @param pet_name pet name
         */
        MyPlayer(string player_name, string pet_name="/dog"): Player(player_name, pet_name){};

        void play(const rwsfi2016_msgs::MakeAPlay& msg)
        {
            //Custom play behaviour. Now I will win the game

            double distance_to_arena = getDistanceToArena();
            ROS_INFO("distance_to_arena = %f", distance_to_arena);

            if (distance_to_arena > 6) //behaviour move to the center of arena
            {
                string arena = "/map";
                move(msg.max_displacement, getAngleToPLayer(arena));
            }
            else //behaviour follow closets prey
            {
                move(msg.max_displacement, getAngleToPLayer(preys_team->players[0]));
            }
        }
};


/**
 * @brief The main function. All you need to do here is enter your name and your pets name
 * @param argc number of command line arguments
 * @param argv values of command line arguments
 * @return result
 */
int main(int argc, char** argv)
{
    // ------------------------
    //Replace this with your name
    // ------------------------
    string my_name = "mriem";
    string my_pet = "/cheetah";

    //initialize ROS stuff
    ros::init(argc, argv, my_name);

    //Creating an instance of class MyPlayer
    MyPlayer my_player(my_name, my_pet);

    //Infinite spinning (until ctrl-c)
    ros::spin();
}
