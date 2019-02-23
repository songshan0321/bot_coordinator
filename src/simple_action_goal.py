#! /usr/bin/env python

from __future__ import print_function
import sys
import rospy, rospkg

# Python wrapper for Firebase
import pyrebase

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the move base action, including the
# goal message and the result message.
import move_base_msgs.msg
from geometry_msgs.msg import PoseStamped

class GoalNavigator():
    def __init__(self):

        self.cycle = 0
        self.current_goal = None
        self.current_waypoints = []
        self.map_points = {"KITCHEN":(0.5,0.5,1.0), "TABLE0":(0.9,1.6,1.0), "TABLE1":(1.1,-0.5,1.0), "TABLE2":(-1.1,-0.5,1.0), "TABLE3":(-1.1,0.5,1.0)}
        self.state = None
        self.result = None
        self.keys = [] # unique firebase keys corresponds to current_waypoints
        self.max_capacity = 3

        self.config = {
        "apiKey": "AIzaSyBAviaNQ--ddkHViwkaJ0fS6EcIC1WuQSg",
        "authDomain": "esc-bot-aff9d.firebaseapp.com",
        "databaseURL": "https://esc-bot-aff9d.firebaseio.com",
        "storageBucket": "esc-bot-aff9d.appspot.com",
        "serviceAccount": rospack.get_path('esc_bot')+"/config/esc-bot-aff9d-20c51ba85efb.json"
        }

        # Initializes a rospy SimpleActionClient
        rospy.init_node('goal_client_py') 
        
        # Creates the SimpleActionClient, passing the type of the action (MoveBaseAction) to the constructor.
        self.client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)

        # Waits until the action server has started up and started listening for goals.
        rospy.loginfo("Waiting for server to start up...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base server")


    def go_to(self, goal):
        """
        Send robot to a specific goal on the map.
        """
        # Sends the goal to the action server.
        self.client.send_goal(goal)
        # Waits for the server to finish performing the action with 60 sec timeout
        self.client.wait_for_result(rospy.Duration(60))
        # Prints out the result of executing the action

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
        delivery_list = db.child("delivery_list").get(user['idToken'])
        # TODO: sort delivery list by timestamp
        # delivery_list = db.child("delivery_list").order_by_child('timestamp').limit_to_first(3).get(user['idToken'])
        if delivery_list.val() != None:
            num_wp = min(len(delivery_list.val()), self.max_capacity)
            for delivery in delivery_list.each():
                if len(self.current_waypoints) >= num_wp:
                    break
                table_name = "TABLE" + str(delivery.val()["table"])
                self.current_waypoints.append(table_name)
                self.keys.append(delivery.key())

    def run_delivery(self):
        if len(self.current_waypoints) > 0 and len(self.current_waypoints) <= self.max_capacity:
            rospy.loginfo("Robot starting delivery cycle %i: %s",self.cycle,self.current_waypoints)
            # Make sure robot is at kitchen
            self.current_goal = self.create_goal(self.map_points["KITCHEN"])
            self.go_to(self.current_goal)
            rospy.loginfo("Robot is at KITCHEN: %s",self.map_points["KITCHEN"])

            rospy.loginfo("Robot loading food.")
            rospy.sleep(3) # Load food for 3 sec

            for i in range(len(self.current_waypoints)):
                # Send a table goal
                self.current_goal = self.create_goal(self.map_points[self.current_waypoints[i]])
                rospy.loginfo("Robot is heading to %s: %s",self.current_waypoints[i], self.map_points[self.current_waypoints[i]])
                self.result = self.go_to(self.current_goal)
                self.state = self.client.get_state()
                if self.state == 3: #SUCCEEDED
                    rospy.loginfo("Robot reached %s: %s",self.current_waypoints[i], self.map_points[self.current_waypoints[i]])
                    rospy.sleep(5) # Serve food for 5 sec
                    # remove elements from delivery list
                    db.child("delivery_list").child(self.keys[i]).remove(user['idToken'])
                elif self.state == 4: #ABORTED
                    db.child("delivery_list").child(self.keys[i]).child('status').set("FAILED")
                    rospy.logwarn("Robot failed to navigate to %s: %s. Hence, delivery reuqest status is set to 'FAILED' and keep as uncleared.",\
                                    self.current_waypoints[i], self.map_points[self.current_waypoints[i]])
            
            # Send robot back to kitchen
            self.current_goal = self.create_goal(self.map_points["KITCHEN"])
            rospy.loginfo("Robot is heading back to KITCHEN: %s",self.map_points["KITCHEN"])
            self.result = self.go_to(self.current_goal)
            self.state = self.client.get_state()
            rospy.loginfo("Robot back to KITCHEN: %s",self.map_points["KITCHEN"])
            rospy.loginfo("Robot had finished a delivery cycle.")

        elif len(self.current_waypoints) == 0:
            rospy.loginfo("No delivery now.")
            rospy.sleep(3)

        else:
            rospy.logerr("Something is WRONG. Robot is trying to send more than %i order(s) at one time.", navigator.max_capacity)
            sys.exit()

if __name__ == '__main__':

    rospack = rospkg.RosPack()
    navigator = GoalNavigator()

    # setup firebase and user authentication
    firebase = pyrebase.initialize_app(navigator.config)
    auth = firebase.auth()
    user = auth.sign_in_with_email_and_password("songshan_you@hotmail.com", "helloworld")
    user = auth.refresh(user['refreshToken'])
    db = firebase.database()
    
    while True:
        try:
            navigator.cycle += 1
            # read waypoints from firebase
            navigator.read_waypoints()
            # start delivery
            navigator.run_delivery()
        
        except rospy.ROSInterruptException:
            print("program interrupted before completion", file=sys.stderr)
            break
                
    