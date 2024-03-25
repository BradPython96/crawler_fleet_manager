import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8, Float32MultiArray
from geometry_msgs.msg import Pose, PoseStamped, PointStamped, PoseArray, Transform, TransformStamped
from tf2_ros import Buffer
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Path
from .test import path_finding
# from actionlib_msgs.msg import GoalStatus
# from visualization_msgs.msg import Marker, MarkerArray
from collections import defaultdict
import numpy as np
import time

# Base class to handle exceptions
from tf2_ros import TransformException 
 
# Stores known frames and offers frame graph requests
from tf2_ros.buffer import Buffer
 
# Easy way to request and receive coordinate frame transform information
from tf2_ros.transform_listener import TransformListener

class GlobalPlanningNode(Node):

    def __init__(self):

        super().__init__('global_planning_node')

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
        self.real_crawler = 1 # 0:False, 1:True
        self.real_crawler_name = "crawler1_pf"
        self.simulated_crawlers = 4
        self.nb_crawlers = self.simulated_crawlers + self.real_crawler

        # Print
        self.print_data_xiao = False
        self.print_data_env = True

        # Timer
        timer_period = 2.0
        self.timer = self.create_timer(timer_period, self.tf_cb)
        self.rate = self.create_rate(1)

        # Publishers
        # self.goals_publisher = self.create_publisher(PointStamped, 'goals', 10)
        # self.goals_list_publisher_ = self.create_publisher(PoseArray, 'goals_list', 10)
        self.crawlers_path_publiser = self.create_publisher(Path, 'crawlers_path', 10)
        self.prio_dict_publisher = self.create_publisher(Float32MultiArray, 'prio_dict', 10)
        self.tf_static_pub = self.create_publisher(TFMessage, 'tf_static', 10)
        # self.nb_crawlers_status_pub =  self.create_publisher(GoalStatus, 'turtlebots_status', 10)
        # self.path_marker_publisher = self.create_publisher(MarkerArray, 'path_marker', 10)

        # Subscribers
        # self.nb_crawlers_sub = self.create_subscription(Int8, 'nb_turtlebots', self.nb_crawlers_cb, 10)
        # self.goals_sub = self.create_subscription(PointStamped, 'clicked_point', self.clicked_point_cb, 10)
        self.mission_command_sub = self.create_subscription(String, 'mission_commands', self.mission_command_cb, 10)
        self.update_prio_real_crawler_sub = self.create_subscription(Float32MultiArray, 'update_prio_real_crawler', self.prio_update_bridge_cb, 10)
        self.update_prio_simulated_crawler_sub = self.create_subscription(Float32MultiArray, 'update_prio_simulated_crawlers', self.prio_update_bridge_cb, 10)
        # tf2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Variables utiles
        self.last_goal_index = -1
        self.goalsList = []
        self.goals_list_msg = PoseArray()
        self.crawler_poses = {}
        self.prio_dict = defaultdict(list)
        self.list_of_wp = []

        self.get_logger().info("Global manager ready !")


    def prio_update_bridge_cb(self,msg):
        self.get_logger().info(f"Prio update : {msg.data}")

    def tf_cb(self):

        trans = None

        if self.real_crawler:
            try:
                now = rclpy.time.Time()
                trans = self.tf_buffer.lookup_transform(
                          "TS0001",
                          self.real_crawler_name,
                          now)
                position = Transform()
                position.translation.y = trans.transform.translation.y+self.X_WORLD_TO_TS0001
                position.translation.z = trans.transform.translation.z+self.Y_WORLD_TO_TS0001
                self.crawler_poses["crawler_0"] = position
            except TransformException as ex:
                self.get_logger().info(f'Could not transform "TS0001" to {self.real_crawler_name}: {ex}')

            tf_static_msg = TFMessage()
            try :
                trans = self.tf_buffer.lookup_transform(
                              "totalstation",
                              "TS0001",
                              now)
                trans_stampd = TransformStamped()
                trans_stampd = trans
                tf_static_msg.transforms.append(trans_stampd)
                self.tf_static_pub.publish(tf_static_msg)
            
            except TransformException as ex:
                self.get_logger().info(f'Could not transform "totalstation" to "TS0001": {ex}')


        for crawler_index in range (1, self.simulated_crawlers+1):
            crawler_name = f"crawler_{crawler_index}/base_link"
            try:
                now = rclpy.time.Time()
                trans = self.tf_buffer.lookup_transform(
                          self.map_frame,
                          crawler_name,
                          now)
                self.crawler_poses[crawler_name.split('/')[0]] = trans.transform
            except TransformException as ex:
                self.get_logger().info(f'Could not transform {self.map_frame} to {crawler_name}: {ex}')


    def mission_command_cb(self,msg):
        

        if (msg.data == 'start'):
            self.get_logger().info('Start mission')
            self.start_mission()

        # elif(msg.data == 'clear'):
        #     self.clear_mission()

        # elif(msg.data == 'reset'):
        #     self.goals_list_msg.poses.clear()
        #     self.list_of_wp.clear()
        #     path_marker_array = MarkerArray()
        #     self.path_marker_publisher.publish(path_marker_array)

        #     #TODO :Faire ça de façon dynamique
        #     # FOR C IN CRAWLERS :
        #         #TODO : Task Reset
        #     self.get_logger().info('Reset mission')

        # elif(msg.data == 'pause'):

        #     # for g in self.goal_handle_list:
        #     #     g.cancel_goal_async()
        #     self.pause_flag = True

        #     #TODO :Faire ça de façon dynamique
        #     # FOR C IN CRAWLERS :
        #         #TODO : Task Pause

        #       self.get_logger().info('Mission paused')
            
        # elif(msg.data == 'run'):
        #     self.get_logger().info('Resume mission')
        #     self.start_mission()

        else:
            self.get_logger().warn("Unknown command. Allowed command(s) : 'start'.")

        return
        
    # def clear_mission(self):
    #     self.goalsList.clear()
    #     Point = PointStamped()
    #     Point.header.frame_id='map'
    #     Point.point.z=10000.0

    #     for i in range(self.nb_crawlers):
    #         self.goals_publisher.publish(Point)

    #     self.get_logger().info('Goal list cleared')
    #     return

    def start_mission(self): 

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

        self.get_logger().info('Computing departs and arrivals')
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
            self.get_logger().info('Departs : ')
            print(departs)
            print("\n")
            self.get_logger().info('Objectifs : ')
            print(arrivals)
            print("\n")
            
        paths, graph = path_finding(self.MAP_DIM, self.OBSTACLES, departs, arrivals)

        if self.print_data_xiao :
            self.get_logger().info("paths : ")
            print(paths)
            print("\n")
            self.get_logger().info("graph : ")
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
            self.get_logger().info("dictionnary : ")
            print(self.prio_dict)
            print("\n")
        
        # We send the dict to the ros GlobalTaskManager node
        prio_dict_msg = Float32MultiArray()
        prio_dict_msg.data=[]
        for point in self.prio_dict:
            prio_dict_msg = Float32MultiArray()
            prio_dict_msg.data=[]
              
            prio_dict_msg.data.append(point[0])
            prio_dict_msg.data.append(point[1])
            for prio in self.prio_dict[point]:
                prio_dict_msg.data.append(np.float32(prio))

            self.prio_dict_publisher.publish (prio_dict_msg) # prio_dict_msg.data = [x,y,prio_list]
        time.sleep(1)


        self.nav_running_flag = True
        self.list_of_wp = []

        for p in range(len(paths)) :
            path = paths[p]
            self.list_of_wp.append([])
            path.pop(0)
            if self.real_crawler and p==0:
                trans=None

                try:
                    now = rclpy.time.Time()
                    trans = self.tf_buffer.lookup_transform(
                              "totalstation",
                              "TS0001",
                              now)
                except TransformException as ex:
                    self.get_logger().info(f'Could not transform TS0001 to crawler_0: {ex}')
                
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


        # path_marker_array = MarkerArray()
        # path_marker_array.markers = []
        for crawler_index in range(self.nb_crawlers): 

            if self.real_crawler:
                crawler_frame = f"crawler_{crawler_index}" 
            else: 
                crawler_frame = f"crawler_{crawler_index+1}"

            path=Path()
            path.header.stamp = self.get_clock().now().to_msg()
            path.header.frame_id = crawler_frame

            # path_marker_c1 = Marker()
            # path_marker_c1.header.frame_id = "map"
            # path_marker_c1.header.stamp = self.get_clock().now().to_msg()
            # path_marker_c1.id = 1
            # path_marker_c1.type = 4
            # path_marker_c1.scale.x = 0.1
            # path_marker_c1.color.b = 1.0
            # path_marker_c1.color.a = 1.0
            # init_p1 = Point()
            # init_p1.x = self.crawler_poses[crawler_frame].pose.position.x
            # init_p1.y = self.crawler_poses[crawler_frame].pose.position.y
            # path_marker_c1.points = [init_p1]

            for p in self.list_of_wp[crawler_index]:

                wp = Pose()
                wp.position = p.pose.position
                wp.orientation = p.pose.orientation

                wp_stamped=PoseStamped()
                wp_stamped.header = path.header
                wp_stamped.pose = wp
                path.poses.append(wp_stamped)

            #     point = Point()
            #     point.x = p.pose.position.x
            #     point.y = p.pose.position.y
            #     point.z = 0.0
            #     path_marker_c1.points.append(point)
            self.crawlers_path_publiser.publish(path)

            if self.print_data_env:
                self.get_logger().info("Path published : ")
                print(f"robot : " + crawler_frame)
                for p in path.poses:
                    print(f"x={p.pose.position.x}, y ={p.pose.position.y}, z={p.pose.position.z}")
                print('\n')

            # path_marker_array.markers.append(path_marker_c1)
            # self.path_marker_publisher.publish(path_marker_array)

        return

    def convert_env2xiao(self, x_env,y_env):
        x_xiao = x_env*self.X_SCALE + self.X_OFFSET
        y_xiao = y_env*self.Y_SCALE + self.Y_OFFSET
        return x_xiao, y_xiao
    
    def convert_xiao2env(self, x_xiao,y_xiao):
        x_env = (x_xiao - self.X_OFFSET)/self.X_SCALE
        y_env = (y_xiao - self.Y_OFFSET)/self.Y_SCALE
        return x_env, y_env


        



def main(args=None):
    rclpy.init(args=args)

    global_planning_node = GlobalPlanningNode()

    rclpy.spin(global_planning_node)

    global_planning_node.destroy_node()
    rclpy.shutdown()
    return

if __name__ == '__main__':
    main()
