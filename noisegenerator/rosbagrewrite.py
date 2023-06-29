#from http://wiki.ros.org/rosbag/Cookbook
import rosbag
import std_msgs
import yaml
# to get commandline arguments
import sys
import os

def checkmultiple(num1,num2):
    for i in range(0,11):
        if(num1/num2) == i: 
            return True
    return False
full_path = os.path.realpath(__file__)
path, filename = os.path.split(full_path)
info_dict = yaml.load(rosbag.Bag(sys.argv[1], 'r')._get_yaml_info())

#http://wiki.ros.org/rosbag/Cookbook
steps=round(info_dict["messages"]/10.0,0)
if steps>1:
    steps=steps-1
print(steps)
writescanman=True

with rosbag.Bag(sys.argv[2], 'w') as outbag:
    h =std_msgs.msg.Header()
    counter=0
    for topic, msg, t in rosbag.Bag(sys.argv[1]).read_messages():
        counter=counter+1
        if checkmultiple(counter,steps):
            print(str(counter/steps*10)+" done")
        # This also replaces tf timestamps under the assumption 
        # that all transforms in the message share the same timestamp
        if topic == "/tf" and msg.transforms:
            outbag.write(topic, msg, msg.transforms[0].header.stamp)
        elif topic == "/tf_static" and msg.transforms:
            outbag.write(topic, msg, msg.transforms[0].header.stamp)
   
        elif topic == "/rosout" or topic == "rosout_agg" or topic == "/rosout_agg":
            #do nothing 
            i=1
        elif msg._has_header:
            outbag.write(topic, msg, msg.header.stamp)
        elif topic == "/clock": 
            h.stamp.secs=msg.clock.secs
            h.stamp.nsecs=msg.clock.nsecs
            outbag.write(topic, msg, h.stamp)