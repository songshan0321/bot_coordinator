import json

person_dict = {"KITCHEN":(0,0,1.0), "TABLE1":(0.8,-6.0,1.0), "TABLE2":(0.8,-9.0,1.0), "TABLE3":(2.8,-6.0,1.0), "TABLE4":(2.8,-9.0,1.0), "TABLE5":(6.8,-6.0,1.0), "TABLE6":(6.8,-9.0,1.0)}
person_json = json.dumps(person_dict)

print(person_json)