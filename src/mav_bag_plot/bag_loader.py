import numpy as np
from scipy.spatial.transform import Rotation as R
import subprocess
import yaml
import os

import rosbag
from rospy import Time

from mav_bag_plot.msg_definitions import Odom, TF, Point, Imu, OpticalFlow
available_msg_types = ["nav_msgs/Odometry",
                       "geometry_msgs/TransformStamped",
                       "geometry_msgs/PointStamped",
                       "sensor_msgs/Imu",
                       "trajectory_msgs/MultiDOFJointTrajectory",
                       "arkflow_ros/OpticalFlow"
                       ]


def find_files(dir, identifier=None):
    paths = []
    for dirpath, dirnames, filenames in os.walk(dir):
        for filename in [f for f in filenames if f.endswith(".bag")]:
            paths.append(os.path.join(dirpath, filename))
            
    if identifier is None:
        pass
    elif isinstance(identifier, str):
        paths = [file for file in paths if identifier in file]
    elif isinstance(identifier, list):
        # checking if any of the identifiers are in the path
        new_paths = []
        for path in paths:
            if any([(i in path) for i in identifier]):
                new_paths.append(path)
            
        paths = new_paths
    
    return paths
    
def load_bags(paths, topics, reset_time = False):
    bags = []
    for path in paths:
        bag_container = BagContainer(path, topics)
        if reset_time:
            bag_container.reset_time()
        bags.append(bag_container)
    return bags
                   
class BagContainer:
    def __init__(self, path, topics):
        self.path = path
        self.bag = rosbag.Bag(path)
        self.topic_dict = {} # topic with list of msgs
        
        print("Loading: ", path)
        self.all_topics = list(self.bag.get_type_and_topic_info()[1].keys())
        
        for topic in topics:
            count = self.all_topics.count(topic)
            
            # look for surrogates if we didn't find the exact ones
            if count == 0:    
                topic = self.find_surrogate(topic)
            msgs = read_topic(self.bag, topic) # TODO extend this with regex (find multiple topics?)
            if msgs is None:
                continue
            self.topic_dict[topic] = msgs
        
        self.loaded_topics = self.topic_dict.keys()
        pass
        
    def get_msgs(self, topic):
        try:
            return self.topic_dict[topic]
        except KeyError:
            #print("Precise topic", topic, " not found, ")
            pass
        try:
            topic_new = self.find_surrogate(topic)
            print("Topic", topic, " replaced with ", topic_new)
            return self.topic_dict[topic_new]
        except KeyError:
            print("Surrogate topic not found")
               
        return None
        
    def find_surrogate(self, topic):
        for full_topic in self.all_topics: # TODO: add all candidates 
            if topic in full_topic:
                #print("Found surrogate: ", full_topic, " for ", topic)
                full_topic = full_topic.strip()
                #print(list(full_topic))
                return full_topic
                
        # return the input if not found
        return topic
    
    def reset_time(self, start_time = None):
        if start_time is None:
            start_time = float("inf")
            for msg_list in self.topic_dict.values():
                next_start_time = msg_list[0].stamp
                if next_start_time < start_time:
                    start_time = next_start_time
                
        print("Start time (first msg):  ", start_time)
        
        for msg_list in self.topic_dict.values():
            for msg in msg_list:
                msg.stamp = msg.stamp - start_time
        

def read_topic(bag, topic):
    #print('Reading topic: '+ topic)
    data = []

    try:
        msg_type = bag.get_type_and_topic_info()[1][topic][0]
    except KeyError:
        print("Oops!  Topic '" + topic +  "' not found, skipping...")
        msg_type = "not_found_in_bag"
        return None
        
    if not msg_type in available_msg_types:
        print("read " + msg_type + " is not implemented yet")
        return None

    for topic, msg, t in bag.read_messages(topics=[topic]):
        time = msg.header.stamp.secs + (10**-9 * msg.header.stamp.nsecs)
                
        if msg_type == "nav_msgs/Odometry":
            element = Odom(msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear, msg.twist.twist.angular, time)
        elif msg_type == "geometry_msgs/TransformStamped":
            element = TF(msg.transform.translation, msg.transform.rotation, time)
        elif msg_type == "geometry_msgs/PointStamped":
            element = Point(msg.point, time)
        elif msg_type == "sensor_msgs/Imu":
            element = Imu(msg.orientation, msg.angular_velocity, msg.linear_acceleration, time)
        elif msg_type == "trajectory_msgs/MultiDOFJointTrajectory":
            if len(msg.points) == 0:
                continue
                print("no points for MultiDOFJointTrajectory")
            elif len(msg.points) == 1:
                element = TF(msg.points[0].transforms[0].translation, msg.points[0].transforms[0].rotation, time)
            else:
                print("Missing implementation on MultiDOFJointTrajectory (multiple points)")
                break
        elif msg_type == "arkflow_ros/OpticalFlow":
            element = OpticalFlow([msg.flow_integral_x, msg.flow_integral_y], [msg.rate_gyro_integral_x, msg.rate_gyro_integral_y], msg.range, msg.integration_interval, time)
        else:
            print("sth wrong with msg_type")
            break
            

        data.append(element)

    print("For topic '" + topic + "' available msgs: " + str(len(data)))
    return data

def get_start_time(bag_path):
      info_dict = yaml.safe_load(subprocess.Popen(['rosbag', 'info', '--yaml', bag_path], stdout=subprocess.PIPE).communicate()[0])
      duration = info_dict['duration']
      start_time = info_dict['start']
      return start_time
