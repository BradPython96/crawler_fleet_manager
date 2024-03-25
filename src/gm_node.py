#!/usr/bin/env python3
import rospy
from math import *
import numpy as np
from collections import defaultdict

from test import path_finding

from std_msgs.msg import String, Empty

from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Transform

from tf2_ros import Buffer,TransformListener, LookupException, ConnectivityException, ExtrapolationException

from task_manager_crawler.srv import StartMissionWP,UpdatePrio,GetPrio, GetPrioResponse, UpdatePrioResponse
from task_manager_crawler.msg import Waypoint
from task_manager_lib.TaskClient import *



class GlobalManager :
    def __init__(self):
        rospy.init_node('global_manager')
        rospy.Service('update_prio', UpdatePrio, self.handle_update_prio)
        rospy.Service('get_prio', GetPrio, self.handle_get_prio)

        # rospy.Subscriber('start_mission', Empty, self.start_mission_cb)
        
        self.gtm_loginfo_pub = rospy.Publisher('gtm_loginfo', String, queue_size=10)
        self.mission_starter_clients={}
        self.prio_dict = {}

        # TF
        self.tfBuffer = Buffer()
        self.tf_listener = TransformListener(self.tfBuffer)
        self.sleep_time = rospy.Duration(2.0)

        # Constantes MAP
        self.MAP_DIM=[5000,1000]
        self.OBSTACLES=[[(450,120),(550,120),(550,875),(450,875)]]
        self.X_SCALE = 100
        self.Y_SCALE = -100
        self.X_OFFSET = 0
        self.Y_OFFSET = 1000
        self.X_WORLD_TO_TS0001=0
        self.Y_WORLD_TO_TS0001=0
        self.map_frame = "world"
        
        # Number of crawlers
        self.real_crawler = 0    # 0:False, 1:True
        self.real_crawler_name = "crawler1_pf"
        self.simulated_crawlers = 4
        self.nb_crawlers = self.simulated_crawlers + self.real_crawler

        # Print
        self.print_data_xiao = False
        self.print_data_env = True

        # Variables utiles
        self.goals_list_msg = PoseArray()
        self.crawler_poses = {}
        self.crawlers_paths = {}
        self.prio_dict = defaultdict(list)
        self.list_of_wp = []

        rospy.sleep(0.5)
        self.gtm_loginfo_pub.publish("GlobalManager ready.")
        rospy.loginfo("GlobalManager ready.\n")

        self.start_mission()

    def start_mission(self):

        rospy.loginfo("Starting mission !\n")

        waiting_for_tf = True

        while  waiting_for_tf :
            waiting_for_tf = False
            if self.real_crawler:
                try:
                    trans = self.tfBuffer.lookup_transform("TS0001",self.real_crawler_name, rospy.Time())
                    position = Transform()
                    position.translation.y = trans.transform.translation.y+self.X_WORLD_TO_TS0001
                    position.translation.z = trans.transform.translation.z+self.Y_WORLD_TO_TS0001
                    self.crawler_poses["crawler_0"] = position

                except (LookupException, ConnectivityException, ExtrapolationException) :
                    self.gtm_loginfo_pub.publish("Could not transform TS0001 to " + self.real_crawler_name + ".")
                    rospy.logwarn("Could not transform TS0001 to " + self.real_crawler_name + " .")
                    waiting_for_tf = True
            
            for crawler_index in range (1, self.simulated_crawlers+1):
                crawler_name = f"crawler_{crawler_index}/base_link"
                try:
                    trans = self.tfBuffer.lookup_transform(
                              self.map_frame,
                              crawler_name,
                              rospy.Time())
                    self.crawler_poses[crawler_name.split('/')[0]] = trans.transform
                    

                except (LookupException, ConnectivityException, ExtrapolationException) :
                    self.gtm_loginfo_pub.publish("Could not transform "+ self.map_frame + " to " + crawler_name + ".")
                    rospy.logwarn("Could not transform "+ self.map_frame + " to " + crawler_name + ".")
                    waiting_for_tf = True

            if waiting_for_tf:
                rospy.sleep(self.sleep_time)

        rospy.loginfo("Transforms found.\n")


        ### TEST AVEC POINTS OBJECTIFS ET nb_crawlers HARDCODES ###

        self.goals_list_msg = PoseArray()

        p1 = Pose()
        p1.position.x = 10.0
        p1.position.y = 7.0
        self.goals_list_msg.poses.append(p1)

        p2 = Pose()
        p2.position.x = 17.0
        p2.position.y = 6.5
        self.goals_list_msg.poses.append(p2)

        p3 = Pose()
        p3.position.x = 30.0
        p3.position.y = 8.0
        self.goals_list_msg.poses.append(p3)

        p4 = Pose()
        p4.position.x = 7.0
        p4.position.y = 5.0
        self.goals_list_msg.poses.append(p4)

        if self.real_crawler:
            p5 = Pose()
            p5.position.x = 2.7
            p5.position.y = 0.7
            self.goals_list_msg.poses.append(p5)

        ###
            
        departs = []

        self.prio_dict = defaultdict(list)

        rospy.loginfo('Computing departs and arrivals')
        for crawler_index in range(self.nb_crawlers):
            if self.real_crawler:
                crawler_name = f"crawler_{crawler_index}"
            else :
                crawler_name = f"crawler_{crawler_index+1}"
            
            initial_pose = self.crawler_poses[crawler_name]
            x_xiao, y_xiao = self.convert_env2xiao(initial_pose.translation.y, initial_pose.translation.z)

            departs.append((int(x_xiao), int(y_xiao)))


        arrivals = []
        for goal in self.goals_list_msg.poses:
            x_xiao, y_xiao = self.convert_env2xiao(goal.position.x, goal.position.y)
            arrivals.append((int(x_xiao), int((y_xiao))))

        if self.print_data_xiao :
            rospy.loginfo('Departs : ')
            print(departs)
            print("\n")
            rospy.loginfo('Objectifs : ')
            print(arrivals)
            print("\n")
            
        paths, graph = path_finding(self.MAP_DIM, self.OBSTACLES, departs, arrivals)

        if self.print_data_xiao :
            rospy.loginfo("paths : ")
            print(paths)
            print("\n")
            rospy.loginfo("graph : ")
            print(graph)
            print("\n")

        # Create Prio dictionnary
        for o in self.OBSTACLES :
            for p in o:
                x_env, y_env = self.convert_xiao2env(p[0],p[1])
                self.prio_dict[(x_env,y_env)] = []
                for i in range (self.nb_crawlers):
                    for k in graph :
                        if int(k[0])==i and (int(k[1][0]), int(k[1][1])) == p:
                            prio = len(graph[i,p])-1

                            while(len(self.prio_dict[(x_env,y_env)])<=prio):
                                self.prio_dict[(x_env,y_env)].append(-1)
                            if self.real_crawler :
                                self.prio_dict[(x_env,y_env)][prio] = i
                            else :
                                self.prio_dict[(x_env,y_env)][prio] = i+1

        if self.print_data_env :
            rospy.loginfo("dictionnary : ")
            print(self.prio_dict)
            print("\n")


        self.nav_running_flag = True
        self.list_of_wp = []

        for p in range(len(paths)) :
            path = paths[p]
            self.list_of_wp.append([])
            path.pop(0)
            if self.real_crawler and p==0:
                trans=None
                try:
                    trans = self.tfBuffer.lookup_transform(
                              "totalstation",
                              "TS0001",
                              rospy.Time())
                except (LookupException, ConnectivityException, ExtrapolationException):
                    rospy.loginfo(f'Could not transform totalstation to TS0001.')
                
                for point in path:
                    x_poin_in_world,y_poin_in_world = self.convert_xiao2env(point[0],point[1])

                    new_wp = PoseStamped()
                    new_wp.header=self.goals_list_msg.header
                    new_wp.pose.position.y=trans.transform.translation.y + (x_poin_in_world - self.X_WORLD_TO_TS0001)
                    new_wp.pose.position.z=trans.transform.translation.z + (y_poin_in_world - self.Y_WORLD_TO_TS0001)
                    self.list_of_wp[p].append(new_wp)
                
            else :
                for point in path :
                    new_wp = PoseStamped()
                    new_wp.header=self.goals_list_msg.header
                    new_wp.pose.position.x, new_wp.pose.position.y=self.convert_xiao2env(point[0],point[1])
                    self.list_of_wp[p].append(new_wp)

        
        
        for crawler_index in range(self.nb_crawlers): 

            if self.real_crawler:
                crawler_frame = f"crawler_{crawler_index}" 
            else: 
                crawler_frame = f"crawler_{crawler_index+1}"

            path=Path()
            path.header.stamp = rospy.Time()
            path.header.frame_id = crawler_frame

            for p in self.list_of_wp[crawler_index]:

                wp = Pose()
                wp.position = p.pose.position
                wp.orientation = p.pose.orientation

                wp_stamped=PoseStamped()
                wp_stamped.header = path.header
                wp_stamped.pose = wp
                path.poses.append(wp_stamped)

            self.crawlers_paths[crawler_frame] = path



            if self.print_data_env:
                rospy.loginfo("New path added : ")
                print(f"robot : " + crawler_frame)
                for p in path.poses:
                    print(f"x={p.pose.position.x}, y ={p.pose.position.y}, z={p.pose.position.z}")
                print('\n')


        for crawler_id in self.crawlers_paths:
            service_name = "/"+str(crawler_id)+"/start_mission_wp"
            waypoints=[]
            self.gtm_loginfo_pub.publish(f"Mission received for {crawler_id} with waypoins : ")
            rospy.loginfo(f"Mission received for {crawler_id} with waypoins : ")
            for path in self.crawlers_paths[crawler_id].poses:
                wp = Waypoint()
                wp.x=path.pose.position.x
                wp.y=path.pose.position.y
                wp.z=path.pose.position.z
                waypoints.append(wp)
                self.gtm_loginfo_pub.publish(f"x={wp.x}, y={wp.y}, z={wp.z}")
                rospy.loginfo(f"x={wp.x}, y={wp.y}, z={wp.z}")  
            rospy.wait_for_service(service_name)

            # Create waypoint_follower service if it doesn't already exist 
            if not crawler_id in self.mission_starter_clients:
                self.mission_starter_clients[crawler_id] = rospy.ServiceProxy(service_name, StartMissionWP)

            # Send request
            try :
                response = self.mission_starter_clients[crawler_id](crawler_id, waypoints)
                if response.success:
                    rospy.loginfo(f"Mission started successfully for {crawler_id}")
                    self.gtm_loginfo_pub.publish(f"Mission started successfully for { crawler_id}")
                else:
                    rospy.logerr(f"Failed to start mission for {crawler_id}")
                    self.gtm_loginfo_pub.publish(f"Failed to start mission for robot {crawler_id}")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed : {e}")
                self.gtm_loginfo_pub.publish(f"Service call failed : {e}")
            print('\n')
        return
    

    def convert_env2xiao(self, x_env,y_env):
        x_xiao = x_env*self.X_SCALE + self.X_OFFSET
        y_xiao = y_env*self.Y_SCALE + self.Y_OFFSET
        return x_xiao, y_xiao
    
    def convert_xiao2env(self, x_xiao,y_xiao):
        x_env = (x_xiao - self.X_OFFSET)/self.X_SCALE
        y_env = (y_xiao - self.Y_OFFSET)/self.Y_SCALE
        return x_env, y_env

    
    def handle_get_prio(self, req):
        rospy.loginfo("Received GetPrio request : ")
        rospy.loginfo(req)
        X=np.round(req.x,2)
        Y=np.round(req.y,2)
        resp=[]
        if (X,Y) in self.prio_dict :
            resp = self.prio_dict[(X,Y)]
            rospy.loginfo(f"Returned priority list at ({X},{Y}) : [" + str(self.prio_dict[(X,Y)])[1:-1] + "]")
            self.gtm_loginfo_pub.publish(f"Returned priority list at ({X},{Y}) : [" + str(self.prio_dict[(X,Y)])[1:-1] + "]")
        else:
            rospy.loginfo(f"No priority at ({X},{Y})")
            self.gtm_loginfo_pub.publish(f"No priority at ({X},{Y})")
        return GetPrioResponse(resp)
    
    def handle_update_prio(self,req):
        X = np.round(req.x,2)
        Y = np.round(req.y,2)
        crawler_id = req.robot_id

        if (X,Y) in self.prio_dict and self.prio_dict[(X,Y)] != []:
                if self.prio_dict[(X,Y)][-1] == crawler_id:
                    self.prio_dict[(X,Y)].pop()
        
                self.gtm_loginfo_pub.publish(f"Priority updated for point ({X},{Y}), with crawler id : {crawler_id}.")
                self.gtm_loginfo_pub.publish(f"New priority dictionnary : {self.prio_dict}")
                rospy.loginfo(f"Priority updated for point ({X},{Y}), with crawler id : {crawler_id}.\nNew priority dictionnary : {self.prio_dict}")
        return UpdatePrioResponse(True)


if __name__ == "__main__":
    gm = GlobalManager()
    rospy.spin()

    
