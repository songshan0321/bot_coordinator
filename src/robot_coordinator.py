#! /usr/bin/env python

from __future__ import print_function
import sys
import rospy, rospkg
import json

# Python wrapper for Firebase
import pyrebase

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the move base action, including the
# goal message and the result message.
import move_base_msgs.msg
from geometry_msgs.msg import PoseStamped

class RobotCoordinator():
    def __init__(self):

        self.rospack = rospkg.RosPack()

        self.config = {
        "apiKey": "AIzaSyCkNjSw6fyvpSB2pjbJgbrg9CcF0x9Njt0",
        "authDomain": "orderfood-b7bbb.firebaseapp.com",
        "databaseURL": "https://orderfood-b7bbb.firebaseio.com",
        "storageBucket": "orderfood-b7bbb.appspot.com",
        "serviceAccount": self.rospack.get_path('esc_bot')+"/config/orderfood-b7bbb-f90f0ee40141.json"
        }
        
        # setup firebase and user authentication
        firebase = pyrebase.initialize_app(self.config)
        auth = firebase.auth()
        self.user = auth.sign_in_with_email_and_password("songshan_you@hotmail.com", "helloworld")
        self.user = auth.refresh(self.user['refreshToken'])
        self.db = firebase.database()

        self.cycle = 0
        self.current_goal = None
        self.current_waypoints = []
        self.map_points = {"KITCHEN":(0,0,1.0), "TABLE1":(0.8,-6.0,1.0), "TABLE2":(0.8,-9.0,1.0), "TABLE3":(2.8,-6.0,1.0), "TABLE4":(2.8,-9.0,1.0), "TABLE5":(6.8,-6.0,1.0), "TABLE6":(6.8,-9.0,1.0)}
        self.state = None
        self.result = None
        self.keys = [] # unique firebase keys corresponds to current_waypoints
        self.max_capacity = 3
        self.delivery_list_name = "delivery_list"

        # Initializes a rospy SimpleActionClient
        rospy.init_node('goal_client_py') 
        
        # Creates the SimpleActionClient, passing the type of the action (MoveBaseAction) to the constructor.
        self.client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)

        # Waits until the action server has started up and started listening for goals.
        rospy.loginfo("Waiting for server to start up...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base server")

    def load_map_points(self,filename):
        """
        Load map points from JSON file
        """
        file_path = self.rospack.get_path('esc_bot')+"/maps/" + filename + ".json"
        with open(file_path, 'r') as f:
            self.map_points = json.load(f)

    def set_delivery_list_name(self, list):
        self.delivery_list_name = list

    def go_to(self, goal):
        """
        Send robot to a specific goal on the map.
        """
        # Sends the goal to the action server.
        self.client.send_goal(goal)
        # Waits for the server to finish performing the action with 100 sec timeout
        self.client.wait_for_result(rospy.Duration(80))
        # Get the result of executing the action
        self.result = self.client.get_result()  # A MoveBaseActionResult

    def create_goal(self, (x,y,theta)):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = theta

        goal = move_base_msgs.msg.MoveBaseGoal()
        goal.target_pose = pose

        return goal

    def read_waypoints(self):
        """
        Read waypoints from firebase
        """
        self.current_waypoints = []
        self.keys = []
        delivery_list = self.db.child(self.delivery_list_name).get(self.user['idToken'])

        if delivery_list.val() != None:
            # convert Pyrebase object into dictionary
            delivery_dict = {}
            for delivery in delivery_list.each():
                delivery_dict[delivery.key()] = delivery.val()

            num_wp = min(len(delivery_dict), self.max_capacity)

            for key in sorted(delivery_dict.iterkeys()):  # Sort order based on unique key
                if len(self.current_waypoints) >= num_wp:
                    break
                # only add if the order is READY to deliver
                if delivery_dict[key]["status"] == "READY":
                    table_name = "TABLE" + str(delivery_dict[key]["table"])
                    self.current_waypoints.append(table_name)
                    self.keys.append(key)
                    self.db.child(self.delivery_list_name).child(key).child('status').set("DELIVERING",self.user['idToken'])

    def run_delivery(self):
        if len(self.current_waypoints) > 0 and len(self.current_waypoints) <= self.max_capacity:
            # set robot status to "WORKING"
            self.db.child("robot_status").set("WORKING",self.user['idToken'])
            rospy.loginfo("Robot starting delivery cycle %i: %s",self.cycle,self.current_waypoints)
            # Make sure robot is at kitchen
            self.current_goal = self.create_goal(self.map_points["KITCHEN"])
            self.go_to(self.current_goal)
            self.state = self.client.get_state()
            rospy.loginfo("Navigation state: " + str(self.state))
            if self.state == 3: #SUCCEEDED
                rospy.loginfo("Robot is at KITCHEN: %s",self.map_points["KITCHEN"])
                rospy.loginfo("Robot loading food.")
            else: #ABORTED or REJECTED or other status
                # set robot status to "FAILED"
                self.db.child("robot_status").set("FAILED",self.user['idToken'])
                for i in range(len(self.current_waypoints)):
                    self.db.child(self.delivery_list_name).child(self.keys[i]).child('status').set("FAILED",self.user['idToken'])
                    rospy.logwarn("Robot failed to navigate to %s: %s. Hence, order status is set to 'FAILED'.",\
                                    self.current_waypoints[i], self.map_points[self.current_waypoints[i]])
                return None

            rospy.sleep(3) # Load food for 3 sec

            for i in range(len(self.current_waypoints)):
                # Send a table goal
                self.current_goal = self.create_goal(self.map_points[self.current_waypoints[i]])
                rospy.loginfo("Robot is heading to %s: %s",self.current_waypoints[i], self.map_points[self.current_waypoints[i]])
                self.result = self.go_to(self.current_goal)
                self.state = self.client.get_state()
                rospy.loginfo("Navigation state: " + str(self.state))
                if self.state == 3: #SUCCEEDED
                    rospy.loginfo("Robot reached %s: %s",self.current_waypoints[i], self.map_points[self.current_waypoints[i]])
                    rospy.sleep(3) # Serve food for 3 sec
                    # set order status to "DELIVERED"
                    self.db.child(self.delivery_list_name).child(self.keys[i]).child('status').set("DELIVERED",self.user['idToken'])
                else: #ABORTED or REJECTED or other status
                    for j in range(i,len(self.current_waypoints)): # set current "DELIVERING" orders to "FAILED"
                        self.db.child(self.delivery_list_name).child(self.keys[j]).child('status').set("FAILED",self.user['idToken'])
                        rospy.logwarn("Robot failed to navigate to %s: %s. Hence, order status is set to 'FAILED'.",\
                                        self.current_waypoints[j], self.map_points[self.current_waypoints[j]])
                    # set robot status to "FAILED"
                    self.db.child("robot_status").set("FAILED",self.user['idToken'])
                    return None
                    
            
            # Send robot back to kitchen
            self.current_goal = self.create_goal(self.map_points["KITCHEN"])
            rospy.loginfo("Robot is heading back to KITCHEN: %s",self.map_points["KITCHEN"])
            self.result = self.go_to(self.current_goal)
            self.state = self.client.get_state()
            rospy.loginfo("Navigation state: " + str(self.state))
            if self.state == 3: #SUCCEEDED
                rospy.loginfo("Robot back to KITCHEN: %s",self.map_points["KITCHEN"])
                # set robot status to "READY"
                self.db.child("robot_status").set("READY",self.user['idToken'])
                rospy.loginfo("Robot had finished a delivery cycle.")
            else: #ABORTED or REJECTED or other status
                rospy.logwarn("Robot failed to back to KITCHEN: %s",self.map_points["KITCHEN"])
                # set robot status to "FAILED"
                self.db.child("robot_status").set("FAILED",self.user['idToken'])
                rospy.loginfo("Robot had partially finished a delivery cycle.")
                return None

        elif len(self.current_waypoints) == 0:
            rospy.loginfo("No delivery now.")
            rospy.sleep(3)

        else:
            rospy.logerr("Something is WRONG. Robot is trying to send more than %i order(s) at one time.", self.max_capacity)
            sys.exit()

def main(list_name, restaurant, round):
    coordinator = RobotCoordinator()
    coordinator.set_delivery_list_name(list_name)
    coordinator.load_map_points(restaurant)
    
    while coordinator.cycle < round or round < 0:
        try:
            coordinator.cycle += 1
            # read waypoints from firebase
            coordinator.read_waypoints()
            # start delivery
            coordinator.run_delivery()
        
        except rospy.ROSInterruptException:
            print("program interrupted before completion", file=sys.stderr)
            sys.exit()

if __name__ == '__main__':
    if len(sys.argv) > 2:
        sys.exit()
    elif len(sys.argv) == 2:
        restaurant = sys.argv[1]
    else:
        restaurant = "restaurant1"

    list = "delivery_list"
    round = -1 # negative means infinite round
    main(list,restaurant,round)