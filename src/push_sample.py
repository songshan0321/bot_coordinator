#! /usr/bin/env python

# Firebase
import pyrebase
import time

config = {
"apiKey": "AIzaSyBAviaNQ--ddkHViwkaJ0fS6EcIC1WuQSg",
"authDomain": "esc-bot-aff9d.firebaseapp.com",
"databaseURL": "https://esc-bot-aff9d.firebaseio.com",
"storageBucket": "esc-bot-aff9d.appspot.com",
"serviceAccount": "~/catkin_ws/src/esc_bot/config/esc-bot-aff9d-20c51ba85efb.json"
}
firebase = pyrebase.initialize_app(config)

auth = firebase.auth()
#authenticate a user
user = auth.sign_in_with_email_and_password("songshan_you@hotmail.com", "helloworld")

# Get a reference to the database service
db = firebase.database()
for i in range(3):
    delivery = {"status": "ACTIVE", "table": i, "timestamp": time.time()}
    result = db.child("delivery_list").push(delivery, user['idToken'])