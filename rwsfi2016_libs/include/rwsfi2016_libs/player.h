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
#include <rwsfi2016_msgs/MakeAPlay.h>

/* _________________________________
   |                                 |
   |              CODE               |
   |_________________________________| */

#define DEFAULT_TIME 0.05 /*Default time to wait for a transform is 0.1 secs*/

using namespace std;
using namespace ros;
using namespace tf;
using namespace boost;

namespace rwsfi2016_libs ///The namespace of this lib
{

    class Player ///Contains a description and methods for a game player

    {
        public:
            /* _________________________________
               |                                 |
               |          PROPPERTIES            |
               |_________________________________| */
            string name; ///the name of the player
            string pet; ///the name of the animal (which defines the speed of the player)
            shared_ptr<Team> red_team, green_team, blue_team, my_team, hunters_team, preys_team; ///The teams
            double last_max_displacement_received;

            NodeHandle node;///A handle to the ros node
            TransformListener listener; ///the transform listener object
            TransformBroadcaster broadcaster; ///The transform publisher object
            shared_ptr<Subscriber> subscriber; ///A subscriber to the MakeAPlay message published by the referee

            /* _________________________________
               |                                 |
               |  Constructor and other methods  |
               |_________________________________| */
            /**
             * @brief The constructor
             * @param player_name the name of the player
             */
            Player(string player_name, string pet_name = "/dog")
            {
                name = names::remap(player_name); //set the name
                setPetName(names::remap(pet_name));//set the name of the animal

                //Create the three teams
                red_team = (shared_ptr<Team>) new Team("red");
                green_team = (shared_ptr<Team>) new Team("green");
                blue_team = (shared_ptr<Team>) new Team("blue");

                //Assign pointers to my_team, preys_team and hunters_team
                if (red_team->playerBelongsToTeam(name))
                {
                    my_team = red_team;
                    preys_team = green_team;
                    hunters_team = blue_team;
                }
                else if(green_team->playerBelongsToTeam(name))
                {
                    my_team = green_team;
                    preys_team = blue_team;
                    hunters_team = red_team;
                }
                else if(blue_team->playerBelongsToTeam(name))
                {
                    my_team = blue_team;
                    preys_team = red_team;
                    hunters_team = green_team;
                }
                else
                {
                    ROS_ERROR("Player %s does not belong to any team.", name.c_str());
                    shutdown();
                }

                //Print initial report
                my_team->printTeamInfo();
                hunters_team->printTeamInfo();
                preys_team->printTeamInfo();
                ROS_WARN("I am %s, team %s. I will hunt %s but I am a little afraid of %s. My pet is %s", name.c_str(), my_team->name.c_str(), preys_team->name.c_str(), hunters_team->name.c_str(), pet.c_str());

                //Radomize a position inside the arena and warp the player to it
                struct timeval t1;      
                gettimeofday(&t1, NULL);
                srand(t1.tv_usec);
                double X=((((double)rand()/(double)RAND_MAX) ) * 2 -1) * 5 ;
                double Y=((((double)rand()/(double)RAND_MAX) ) * 2 -1) * 5 ;
                warp(X,Y);
                Duration(0.1).sleep(); //sleep a while to fill the tf buffer
                warp(X,Y);

                //initialize the subscriber
                string make_a_play_topic = "/make_a_play" + pet;
                subscriber = (boost::shared_ptr<ros::Subscriber>) new ros::Subscriber;
                *subscriber = node.subscribe(make_a_play_topic, 1, &Player::makeAPlayCallback, this);

                ROS_INFO("Waiting for messages on topic %s ...", make_a_play_topic.c_str());
            }

            /**
             * @brief 
             * @param msg
             */
            void makeAPlayCallback(const rwsfi2016_msgs::MakeAPlay& msg)
            {
                last_max_displacement_received = msg.max_displacement; 

                //Call the virtual method play. Runs the default behaviour defined in PLayer class or a custom behaviour if the class MyPlayer defines the method play()
                play(msg);

            }

            virtual void play(const rwsfi2016_msgs::MakeAPlay& msg)
            {
                ROS_WARN("Default play behaviour. You should implement a play method in your class MyPlayer");
                move(msg.max_displacement, M_PI/30);
            }


            /**
             * @brief Distance from this player to another player p
             * @param p an instance of the other player
             * @return distance in meters
             */
            double getDistanceToPlayer(string other_player, double time_to_wait=DEFAULT_TIME)
            {
                StampedTransform t; //The transform object
                //Time now = Time::now(); //get the time
                Time now = Time(0); //get the latest transform received

                try //get the transformation between both players
                {
                    listener.waitForTransform(name, other_player, now, Duration(time_to_wait));
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
                //Time now = Time::now(); //get the time
                Time now = Time(0); //get the latest transform received

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
                //Time now = Time::now(); //get the time
                Time now = Time(0); //get the latest transform received

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
                if (displacement > last_max_displacement_received)
                {
                    ROS_WARN("%s, you cannot move more than %0.2f and you asked to move %0.2f. Are you trying to cheat? As a penalty, this time you will not move!", name.c_str(), last_max_displacement_received, displacement);
                    displacement = 0;
                }

                if (isnan(turn_angle))
                {
                    ROS_WARN("%s, angle given is nan. Using 0 instead", name.c_str());
                    turn_angle = 0;
                }

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

            double getDistanceToArena(void)
            {
                return getDistanceToPlayer("/map");
            }



            void warp(double X, double Y)
            {
                Quaternion q; 
                q.setRPY(0, 0, 0);

                Transform t;
                t.setOrigin( Vector3(X, Y, 0.0) );
                t.setRotation(q);
                broadcaster.sendTransform(StampedTransform(t, ros::Time::now(), "/map", name));
            }


            void setPetName(string pet_name)
            {
                if (pet_name == "/cheetah" || pet_name == "/dog" ||pet_name == "/cat" ||pet_name == "/turtle")
                {
                    pet = pet_name; 
                }
                else 
                {
                    ROS_ERROR("%s does not exist. You must choose a pet from: cheetah, dog, cat or turtle", pet_name.c_str());
                    shutdown();
                }
            }



    };//end of class Player
};//end of namespace rwsfi_libs


#endif
