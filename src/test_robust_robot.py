#! /usr/bin/env python

import pytest
import pyrebase
import time
import rospkg
import os
import signal
import subprocess

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
    result = db.child("test_list").remove(user['idToken'])
    return result

def setup_module(module):
    """
    Setup before module
    """
    print()
    print("-------------- setup before module --------------")
    cmd = "roslaunch esc_bot esc_bot_2_blocked.launch"
    pro = subprocess.Popen(cmd, stdout=subprocess.PIPE, 
                       shell=True, preexec_fn=os.setsid)
    time.sleep(5)

def teardown_module(module):
    """
    Teardown after module
    """
    print("-------------- teardown after module --------------")
    os.system("killall -9 gzserver gzclient & rosnode kill -a")
    print("Sleep 10 sec")
    time.sleep(10)

def pytest_keyboard_interrupt(excinfo):
    teardown_module()

##################################### Test cases ############################################

# Robustness Test
def test_path_blocked():
    """
    Robustness Test: Test robot status and orders' status are changed to "FAILED" when it failed to navigate to destination
    """
    print("Robustness Test: test_path_blocked")
    db, user = setup_database()
    result = clean_order(db,user)

    # Send 1 food order
    num_order = 2
    for i in range(num_order):
        delivery = {"food": "Chicken Rice",\
                    "status": "READY",\
                    "table": i+1,\
                    "timestamp": time.time()}
        result = db.child("test_list").push(delivery, user['idToken'])
    print("Sent {} orders".format(num_order))

    time.sleep(2)

    main("test_list","restaurant2",1)

    # Check robot status
    robot_status = db.child("robot_status").get(user['idToken'])
    assert robot_status.val() == "FAILED"
    # Check order status
    test_list = db.child("test_list").get(user['idToken'])
    for order in test_list.each():
        assert order.val()["status"] == "FAILED"
