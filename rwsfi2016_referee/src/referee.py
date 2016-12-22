#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
from rwsfi2016_msgs.msg import MakeAPlay
import random
import tf
import math
import os
import subprocess

import rospkg
from sensor_msgs.msg import Image, PointCloud2, PointField

score = [0,0,0]
pub_cheetah = rospy.Publisher('make_a_play/cheetah', MakeAPlay, queue_size=0)
pub_dog = rospy.Publisher('make_a_play/dog', MakeAPlay, queue_size=0)
pub_cat = rospy.Publisher('make_a_play/cat', MakeAPlay, queue_size=0)
pub_turtle = rospy.Publisher('make_a_play/turtle', MakeAPlay, queue_size=0)

pub_referee = rospy.Publisher("referee_markers", MarkerArray, queue_size=10)
pub_score = rospy.Publisher("score_markers", MarkerArray, queue_size=10)
pub_rip = rospy.Publisher("kill_markers", MarkerArray, queue_size=10)
pub_killer = rospy.Publisher("victim", String, queue_size=10)

rate = 0
game_duration = rospy.get_param('/game_duration')
positive_score = rospy.get_param('/positive_score')
negative_score = rospy.get_param('/negative_score')
killed = []
teamA = []
teamB = []
teamC = []

pause_game = 0

def queryCallback(event):

    #get a complete list of alive players
    print("This is a query callback")
    global pause_game
    pause_game = 1

    a = MakeAPlay()
    global teamA, teamB, teamC
    global killed
    print("killed: " + str(killed))

    if len(killed)==0:
        players_killed = []
    else:
        players_killed = [i[0] for i in killed]

    for player in teamA:
        if player in players_killed:
            a.red_dead.append(player)
        else:
            a.red_alive.append(player)

    for player in teamB:
        if player in players_killed:
            a.green_dead.append(player)
        else:
            a.green_alive.append(player)

    for player in teamC:
        if player in players_killed:
            a.blue_dead.append(player)
        else:
            a.blue_alive.append(player)

    players = a.red_alive + a.green_alive + a.blue_alive
    print("list of players alive = " + str(players))

    desgracado = random.choice(players)
    print("choose " + str(desgracado) + " as desgracado")


    #publish the point cloud and wait a bit
    objects = ["banana", "tomato", "onion", "soda_can"]
    chosen_object = random.choice(objects)


    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    # list all packages, equivalent to rospack list
    #rospack.list_pkgs() 
    # get the file path for rospy_tutorials
    path_pcds = rospack.get_path('rwsfi2016_referee') + "/../pcd/"

    cmd = "rosrun rwsfi2016_referee pcd2pointcloud _input:=" + path_pcds + chosen_object + ".pcd _output:=/object_point_cloud /map:=/" + desgracado + " _one_shot:=1"
    print "Executing command: " + cmd
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    for line in p.stdout.readlines():
        print line,
        p.wait()

    #write here the service call for the /desgracado/game_query
    #... to be continued tomorrow ...


    
    #game can be resumed
    pause_game = 0


def timerCallback(event):

    if pause_game:
        return

    a = MakeAPlay()

    global teamA, teamB, teamC
    global killed
    print("killed: " + str(killed))

    if len(killed)==0:
        players_killed = []
    else:
        players_killed = [i[0] for i in killed]

    for player in teamA:
        if player in players_killed:
            a.red_dead.append(player)
        else:
            a.red_alive.append(player)

    for player in teamB:
        if player in players_killed:
            a.green_dead.append(player)
        else:
            a.green_alive.append(player)

    for player in teamC:
        if player in players_killed:
            a.blue_dead.append(player)
        else:
            a.blue_alive.append(player)

    #cheetah
    a.max_displacement = random.random()/10
    if not rospy.is_shutdown():
        global pub_cheetah
        pub_cheetah.publish(a)

    a.max_displacement = random.random()/10
    if not rospy.is_shutdown():
        global pub_dog
        pub_dog.publish(a)

    a.max_displacement = random.random()/10
    if not rospy.is_shutdown():
        global pub_cat
        pub_cat.publish(a)

    a.max_displacement = random.random()/10
    if not rospy.is_shutdown():
        global pub_turtle
        pub_turtle.publish(a)


def gameEndCallback(event):

    rospy.loginfo("Game finished")
    global pub_score

    ma = MarkerArray()

    m1 = Marker()
    m1.header.frame_id = "/map"
    m1.type = m1.TEXT_VIEW_FACING
    m1.action = m1.ADD
    m1.id = 777;
    m1.scale.x = 0.2
    m1.scale.y = 0.2
    m1.scale.z = 0.9
    m1.color.a = 1.0
    m1.color.r = 0.1
    m1.color.g = 0.1
    m1.color.b = 0.1
    m1.pose.position.x = .5
    m1.pose.position.y = 4.5
    m1.pose.position.z = 0 

    if score[0] > score[1] and score[0] > score[2]:
        m1.text = "Team R wins the game"
    elif score[1] > score[0] and score[1] > score[2]:
        m1.text = "Team G wins the game"
    elif score[2] > score[0] and score[2] > score[1]:
        m1.text = "Team B wins the game"

    ma.markers.append(m1)
    pub_score.publish(ma)
    rate.sleep()

    rospy.signal_shutdown("Game finished")

def talker():
    rospy.init_node('referee', anonymous=True)
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()
    global rate 
    rate = rospy.Rate(2) # 10hz
    rate.sleep()

    hunting_distance = rospy.get_param('/hunting_distance')
    global teamA
    teamA = rospy.get_param('/red')
    global teamB
    teamB = rospy.get_param('/green')
    global teamC
    teamC = rospy.get_param('/blue')

    rospy.Timer(rospy.Duration(0.1), timerCallback, oneshot=False)
    rospy.Timer(rospy.Duration(game_duration), gameEndCallback, oneshot=True)

    #A query every 5 seconds
    rospy.Timer(rospy.Duration(5), queryCallback, oneshot=False)
    game_start = rospy.get_time()

    while not rospy.is_shutdown():


        print killed
        tic = rospy.Time.now()
        for i in killed:
            d = tic-i[1]
            if d.to_sec() > 10:
                rospy.logwarn("Ressuscitating %s", i[0])
                killed.remove(i)
                broadcaster.sendTransform((random.random()*10 -5, random.random()*10 -5, 0), tf.transformations.quaternion_from_euler(0, 0, 0), tic, i[0], "/map")


        print killed
        rospy.loginfo("Checking ...")

        to_be_killed_A = [];
        to_be_killed_B = [];
        to_be_killed_C = [];
        max_distance_from_center_of_arena = 8

        #Checking if players don't stray from the arena
        for player in teamA:
            #check if hunter is killed
            result = [item for item in killed if item[0] == player]
            if not result:
                try:
                    (trans,rot) = listener.lookupTransform("/map", player, rospy.Time(0))
                    distance = math.sqrt(trans[0]*trans[0] + trans[1]*trans[1])
                    #rospy.loginfo("D from %s to %s is %f", hunter, prey, distance)
                    if distance > max_distance_from_center_of_arena:
                        to_be_killed_A.append(player);
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    t=1
                    #rospy.loginfo("Referee: tf error")

        for player in teamB:
            #check if hunter is killed
            result = [item for item in killed if item[0] == player]
            if not result:
                try:
                    (trans,rot) = listener.lookupTransform("/map", player, rospy.Time(0))
                    distance = math.sqrt(trans[0]*trans[0] + trans[1]*trans[1])
                    #rospy.loginfo("D from %s to %s is %f", hunter, prey, distance)
                    if distance > max_distance_from_center_of_arena:
                        to_be_killed_A.append(player);
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    t=1
                    #rospy.loginfo("Referee: tf error")

        for player in teamC:
            #check if hunter is killed
            result = [item for item in killed if item[0] == player]
            if not result:
                try:
                    (trans,rot) = listener.lookupTransform("/map", player, rospy.Time(0))
                    distance = math.sqrt(trans[0]*trans[0] + trans[1]*trans[1])
                    #rospy.loginfo("D from %s to %s is %f", hunter, prey, distance)
                    if distance > max_distance_from_center_of_arena:
                        to_be_killed_A.append(player);
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    t=1
                    #rospy.loginfo("Referee: tf error")

        #Check if anyone is hunted

        #Team A hunting team B
        for hunter in teamA:
            
            #check if hunter is killed
            result = [item for item in killed if item[0] == hunter]
            if not result:
                for prey in teamB:
                    try:
                        (trans,rot) = listener.lookupTransform(hunter, prey, rospy.Time(0))
                        distance = math.sqrt(trans[0]*trans[0] + trans[1]*trans[1])
                        #rospy.loginfo("D from %s to %s is %f", hunter, prey, distance)
                        if distance < hunting_distance:
                            to_be_killed_A.append(prey);
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        t=1
                        #rospy.loginfo("Referee: tf error")

        #Team A hunting team B
        for hunter in teamB:
            #check if hunter is killed
            result = [item for item in killed if item[0] == hunter]
            if not result:
                for prey in teamC:
                    try:
                        (trans,rot) = listener.lookupTransform(hunter, prey, rospy.Time(0))
                        distance = math.sqrt(trans[0]*trans[0] + trans[1]*trans[1])
                        #rospy.loginfo("D from %s to %s is %f", hunter, prey, distance)
                        if distance < hunting_distance:
                            to_be_killed_B.append(prey);
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        t=1
                        #rospy.loginfo("Referee: tf error")

        #Team C hunting team A
        for hunter in teamC:
            #check if hunter is killed
            result = [item for item in killed if item[0] == hunter]
            if not result:
                for prey in teamA:
                    try:
                        (trans,rot) = listener.lookupTransform(hunter, prey, rospy.Time(0))
                        distance = math.sqrt(trans[0]*trans[0] + trans[1]*trans[1])
                        #rospy.loginfo("D from %s to %s is %f", hunter, prey, distance)
                        if distance < hunting_distance:
                            to_be_killed_C.append(prey);
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        t=1
                        #rospy.loginfo("Referee: tf error")


        ma_killed = MarkerArray()



        kill_time = rospy.Time.now()
        for tbk in to_be_killed_A:
            #rospy.logwarn("%s is to be killed by Team A", tbk)

            #Check if tbk is in the list of killed players
            found = False
            for item in killed:
                if item[0] == tbk:
                    found = True
            
            if found == False:
                killed.append((tbk,kill_time))
                rospy.logwarn("Red hunted %s", tbk)
                score[0] = score[0] + positive_score
                score[1] = score[1] + negative_score
                s = String()
                s.data = tbk
                #pub_killer.publish(s)
                mk1 = Marker()
                mk1.header.frame_id = "/map"
                (trans,rot) = listener.lookupTransform("/map", tbk, rospy.Time(0))
                mk1.pose.position.x = trans[0]
                mk1.pose.position.y = trans[1]
                mk1.type = mk1.TEXT_VIEW_FACING
                mk1.action = mk1.ADD
                mk1.id = 0;
                mk1.ns = tbk
                mk1.scale.z = 0.4
                mk1.color.a = 1.0
                mk1.text = tbk
                mk1.lifetime = rospy.Duration.from_sec(5)
                mk1.frame_locked = 0
                ma_killed.markers.append(mk1)

                broadcaster.sendTransform((-100, -100, 0), tf.transformations.quaternion_from_euler(0, 0, 0), kill_time, tbk, "/map")


            else:
                t=1
                #rospy.logwarn("%s was already hunted", tbk)


        for tbk in to_be_killed_B:
            #rospy.logwarn("%s is to be killed by Team B", tbk)

            #Check if tbk is in the list of killed players
            found = False
            for item in killed:
                if item[0] == tbk:
                    found = True
            
            if found == False:
                killed.append((tbk,kill_time))
                rospy.logwarn("Green hunted %s", tbk)
                score[1] = score[1] + positive_score
                score[2] = score[2] + negative_score
                s = String()
                s.data = tbk
                #pub_killer.publish(s)

                mk1 = Marker()
                mk1.header.frame_id = "/map"
                (trans,rot) = listener.lookupTransform("/map", tbk, rospy.Time(0))
                mk1.pose.position.x = trans[0]
                mk1.pose.position.y = trans[1]
                mk1.type = mk1.TEXT_VIEW_FACING
                mk1.action = mk1.ADD
                mk1.id = 0;
                mk1.ns = tbk
                mk1.scale.z = 0.4
                mk1.color.a = 1.0
                mk1.text = tbk
                mk1.lifetime = rospy.Duration.from_sec(5)
                mk1.frame_locked = 0
                ma_killed.markers.append(mk1)
                broadcaster.sendTransform((100, -100, 0), tf.transformations.quaternion_from_euler(0, 0, 0), kill_time, tbk, "/map")

            else:
                t=1
                #rospy.logwarn("%s was already hunted", tbk)

            
        for tbk in to_be_killed_C:
            #rospy.logwarn("%s is to be killed by Team C", tbk)

            #Check if tbk is in the list of killed players
            found = False
            for item in killed:
                if item[0] == tbk:
                    found = True
            
            if found == False:
                killed.append((tbk,kill_time))
                rospy.logwarn("Blue hunted %s", tbk)
                score[2] = score[2] + positive_score
                score[0] = score[0] + negative_score
                s = String()
                s.data = tbk
                #pub_killer.publish(s)


                mk1 = Marker()
                mk1.header.frame_id = "/map"
                (trans,rot) = listener.lookupTransform("/map", tbk, rospy.Time(0))
                mk1.pose.position.x = trans[0]
                mk1.pose.position.y = trans[1]
                mk1.type = mk1.TEXT_VIEW_FACING
                mk1.action = mk1.ADD
                mk1.id = 0;
                mk1.ns = tbk
                mk1.scale.z = 0.4
                mk1.color.a = 1.0
                mk1.text = tbk
                mk1.lifetime = rospy.Duration.from_sec(5)
                mk1.frame_locked = 0
                ma_killed.markers.append(mk1)
                broadcaster.sendTransform((100, 100, 0), tf.transformations.quaternion_from_euler(0, 0, 0), kill_time, tbk, "/map")

            else:
                t=1
                #rospy.logwarn("%s was already hunted", tbk)
        
        if ma_killed.markers:
            pub_rip.publish(ma_killed)
        #print killed
                        

        #rospy.logwarn("Team A: %s hunted %s", hunter, prey)
        ##os.system('rosnode kill ' + prey)
        #score[0] = score[0] + positive_score
        #score[1] = score[1] + negative_score
        #s = String()
        #s.data = prey
        #pub_killer.publish(s)


        ma = MarkerArray()

        m1 = Marker()
        m1.header.frame_id = "/map"
        m1.type = m1.TEXT_VIEW_FACING
        m1.action = m1.ADD
        m1.id = 0;
        m1.scale.x = 0.2
        m1.scale.y = 0.2
        m1.scale.z = 0.6
        m1.color.a = 1.0
        m1.color.r = 1.0
        m1.color.g = 0.0
        m1.color.b = 0.0
        m1.text = "R=" + str(score[0])
        m1.pose.position.x = -5.0
        m1.pose.position.y = 5.2
        m1.pose.position.z = 0 
        ma.markers.append(m1)

        m2 = Marker()
        m2.header.frame_id = "/map"
        m2.type = m2.TEXT_VIEW_FACING
        m2.action = m2.ADD
        m2.id = 1;
        m2.scale.x = 0.2
        m2.scale.y = 0.2
        m2.scale.z = 0.6
        m2.color.a = 1.0
        m2.color.r = 0.0
        m2.color.g = 1.0
        m2.color.b = 0.0
        m2.text = "G=" + str(score[1])
        m2.pose.position.x = -3.0
        m2.pose.position.y = 5.2
        m2.pose.position.z = 0 
        ma.markers.append(m2)

        m3 = Marker()
        m3.header.frame_id = "/map"
        m3.type = m3.TEXT_VIEW_FACING
        m3.action = m3.ADD
        m3.id = 2;
        m3.scale.x = 0.2
        m3.scale.y = 0.2
        m3.scale.z = 0.6
        m3.color.a = 1.0
        m3.color.r = 0.0
        m3.color.g = 0.0
        m3.color.b = 1.0
        m3.text = "B=" + str(score[2])
        m3.pose.position.x = -1.0
        m3.pose.position.y = 5.2
        m3.pose.position.z = 0 
        ma.markers.append(m3)

        m4 = Marker()
        m4.header.frame_id = "/map"
        m4.type = m4.TEXT_VIEW_FACING
        m4.action = m4.ADD
        m4.id = 3;
        m4.scale.x = 0.2
        m4.scale.y = 0.2
        m4.scale.z = 0.6
        m4.color.a = 1.0
        m4.color.r = 0.0
        m4.color.g = 0.0
        m4.color.b = 1.0
        #,c rospy.Time(

        time_now = rospy.get_time()

        m4.text = "Time " + str(format(time_now-game_start, '.2f')) + " of " + str(game_duration)
        m4.pose.position.x = 3.5
        m4.pose.position.y = 5.2
        m4.pose.position.z = 0 
        ma.markers.append(m4)

        pub_score.publish(ma)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
