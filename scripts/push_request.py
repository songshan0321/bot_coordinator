#! /usr/bin/env python

# Firebase
import pyrebase
import time
import rospkg

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
for i in range(2,12):
    delivery = {"food": "Chicken Rice","status": "READY", "table": i+1, "timestamp": time.time()}
    result = db.child("delivery_list").push(delivery, user['idToken'])