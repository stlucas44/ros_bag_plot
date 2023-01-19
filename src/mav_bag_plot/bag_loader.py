import numpy as np
from scipy.spatial.transform import Rotation as R
import subprocess
import yaml
import os

import rosbag
from rospy import Time
import ros_numpy

from mav_bag_plot.msg_definitions import Odom, TF, Point, Imu
available_msg_types = ["nav_msgs/Odometry",
                       "geometry_msgs/TransformStamped",
                       "geometry_msgs/PointStamped",
                       "sensor_msgs/Imu"
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
    
def load_bags(paths, topics):
    bags = []
    for path in paths:
        bag_container = BagContainer(path, topics)
        bags.append(bag_container)
    return bags                  
                   
class BagContainer:
    def __init__(self, path, topics):
        self.path = path
        self.bag = rosbag.Bag(path)
        self.topic_dict = {} # topic with list of msgs
        
        print("Loading: ", path)
        for topic in topics:
            msgs = read_topic(self.bag, topic)
            if msgs is None:
                continue
            self.topic_dict[topic] = msgs
        
        self.loaded_topics = self.topic_dict.keys()
        pass
    def get_msgs(self, topic):
        try:
            return self.topic_dict[topic]
        except KeyError:
            return None

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
        else:
            print("sth wrong with msg_type")
            break
            

        data.append(element)

    print("For topic '" + topic + "' available msgs: " + str(len(data)))
    return data

def get_start_time(bag_path):
      info_dict = yaml.safe_load(subprocess.Popen(['rosbag', 'info', '--yaml', bag_path], stdout=subprocess.PIPE).communicate()[0])
      duration = info_dict['duration']
      print("Bag_start", info_dict['start'])
      start_time = info_dict['start']
      return start_time
