#! /usr/bin/env python

import pytest
import pyrebase
import time
import rospkg
from robot_coordinator import *

def setup_database():
    """
    Setup connection with firebase
    """
    rospack = rospkg.RosPack()
    config = {
            "apiKey": "AIzaSyCkNjSw6fyvpSB2pjbJgbrg9CcF0x9Njt0",
            "authDomain": "orderfood-b7bbb.firebaseapp.com",
            "databaseURL": "https://orderfood-b7bbb.firebaseio.com",
            "storageBucket": "orderfood-b7bbb.appspot.com",
            "serviceAccount": rospack.get_path('esc_bot')+"/config/orderfood-b7bbb-f90f0ee40141.json"
            }
    firebase = pyrebase.initialize_app(config)

    auth = firebase.auth()
    #authenticate a user
    user = auth.sign_in_with_email_and_password("songshan_you@hotmail.com", "helloworld")

    # Get a reference to the database service
    db = firebase.database()

    return db,user

def clean_order(db,user):
    """
    Clean order list on firebase real-time database
    """
    result = db.child("delivery_list").remove(user['idToken'])
    return result

##################################### Test cases ############################################

def test_read_waypoints_within_max_cap():
    """
    Test delivery is scheduled within maximum robot capacity
    """
    print("test_read_waypoints_within_max_cap")
    db, user = setup_database()
    result = clean_order(db,user)
    
    num_order = 5
    # Send 5 food order
    for i in range(num_order):
        delivery = {"food": "Chicken Rice",\
                    "status": "READY",\
                    "table": i+1,\
                    "timestamp": time.time()}
        result = db.child("delivery_list").push(delivery, user['idToken'])
    print("Sent {} orders".format(num_order))
    time.sleep(2)
    rospack = rospkg.RosPack()
    coordinator = RobotCoordinator()
    coordinator.read_waypoints()
    print ("Waypoints length: {}".format(len(coordinator.current_waypoints)))

    assert len(coordinator.current_waypoints) <= coordinator.max_capacity


def test_read_waypoints_chronogically():
    """
    Test delivery is scheduled chronogically
    """
    print("test_read_waypoints_chronogically")
    db, user = setup_database()
    result = clean_order(db,user)
    
    num_order = 5
    # Send 5 food order
    for i in range(num_order):
        delivery = {"food": "Chicken Rice",\
                    "status": "READY",\
                    "table": i+1,\
                    "timestamp": time.time()}
        result = db.child("delivery_list").push(delivery, user['idToken'])
    print("Sent {} orders".format(num_order))
    time.sleep(2)
    rospack = rospkg.RosPack()
    coordinator = RobotCoordinator()
    coordinator.read_waypoints()
    current_waypoint = 0
    for waypoint in coordinator.current_waypoints:
        if int(waypoint[-1]) > current_waypoint:
            current_waypoint = int(waypoint[-1])
        else:
            print("Waypoints are not sorted chronogically.")
            assert False

    assert True


def test_read_waypoints_only_ready():
    """
    Test delivery scheduled are all in READY state
    """
    print("test_read_waypoints_only_ready")
    db, user = setup_database()
    result = clean_order(db,user)
    
    status_ls = ["ACTIVE","ACCEPT","READY","REJECTED","FAILED"] # only TABLE3 is READY
    num_order = 5
    # Send 5 food order
    for i in range(num_order):
        delivery = {"food": "Chicken Rice",\
                    "status": status_ls[i],\
                    "table": i+1,\
                    "timestamp": time.time()}
        result = db.child("delivery_list").push(delivery, user['idToken'])
    print("Sent {} orders".format(num_order))
    time.sleep(2)
    rospack = rospkg.RosPack()
    coordinator = RobotCoordinator()
    coordinator.read_waypoints()

    for waypoint in coordinator.current_waypoints:
        if int(waypoint[-1]) != 3:
            assert False

    assert True