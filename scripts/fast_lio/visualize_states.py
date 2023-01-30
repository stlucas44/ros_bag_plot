#%matplotlib widget

from datetime import datetime, timedelta
import os
import copy
 
import mav_bag_plot.bag_loader as bag_loader
from mav_bag_plot.msg_plotter import vis_states, vis_odom, vis_quat

min_range = 1.0


def main():
    #compare_updates()    
    #compare_corrupted_bag()
    #compare_updated_bag()
    #cov_comparison()
    bug_fixing()
    
def compare_updates():
    paths = ["/home/lucas/bags/gtsam_fusion/original_prediction_update.bag",
             #"/home/lucas/bags/gtsam_fusion/new_prediction_update.bag",
             #"/home/lucas/bags/gtsam_fusion/new_prediction_update2.bag",
             "/home/lucas/bags/gtsam_fusion/missing_pcl_with_gtsam4.bag",
             "/home/lucas/bags/gtsam_fusion/missing_pcl_with_gtsam4.bag"
             ]
   
    names = [
             "Original",
             #"Try1",
             #"Try2",
             "Try3",
             "CleanedUp4"
             ]
    
    vis_odom(paths, names)
def compare_corrupted_bag():
    paths = [
             "/home/lucas/bags/gtsam_fusion/original.bag",
             "/home/lucas/bags/gtsam_fusion/missing_pcl_no_gps.bag",
             "/home/lucas/bags/gtsam_fusion/missing_pcl_with_gtsam3.bag",
             "/home/lucas/bags/gtsam_fusion/missing_pcl_with_gtsam4.bag"
             ]
   
    names = [
             "Original",
             "Corrupted",
             "GTSAM FB 3",
             "GTSAM FB 4",
             ]
    #vis_states(paths, names)
    vis_odom(paths, names)
    
def compare_updated_bag():
    dir = "/home/lucas/bags/gtsam_fusion/"
    paths = [
             "/home/lucas/bags/gtsam_fusion/original.bag",
             #"/home/lucas/bags/gtsam_fusion/missing_pcl_no_gps.bag",
             "/home/lucas/bags/gtsam_fusion/evaluate_update0.bag",
             "/home/lucas/bags/gtsam_fusion/fix_timing0.bag",
             ]
    names = ["origin (uncorrupted) bag",
             #"standard LIO rerun",
             "Current GTSAM update",
             "Fixed timing"
             ]
             
    topics = ['/kolibri/mav_state_estimator/optimization',
              #'/Gnss',
              '/Odometry']
    topic_names = [": Pose Graph",
                 #": GNSS",
                 ": LIO"]          
    
    identifier = ["original.", "evaluate_update0", "fix_timing0"]
    files = bag_loader.find_files(dir, identifier)
    bags = bag_loader.load_bags(files, topics)
    names = [os.path.basename(file)[:-4] for file in files]
    
    vis_odom(bags, names, topics = topics)
    vis_states(bags, names, topics = topics, topic_names = topic_names)
    
def cov_comparison():
    dir = "/home/lucas/bags/gtsam_fusion/"
    
    identifier = ["original.", 
                  #"no_corruption_with_update",
                  #"with_corruption_with_update",
                 "debug_prediction_cov_0.01_init",
                 # "b395590",
                 # "89ee811",
                 "cov0.001.",
                 "cov0.0001."
                 ]
             
    topics = ['/kolibri/mav_state_estimator/optimization',
              '/Odometry',
              #'/debug/odom'
              ]
    topic_names = [": Pose Graph",
                 ": LIO update",
                 ": LIO debug prediction"]          
        
    files = bag_loader.find_files(dir, identifier)
    bags = bag_loader.load_bags(files, topics)
    names = [os.path.basename(file)[:-4] for file in files]
    
    
    for bag in bags:
        bag.reset_time()
        print(bag.loaded_topics)
        if not "original" in bag.path:
            bag.topic_dict.pop('/kolibri/mav_state_estimator/optimization')
    vis_odom(bags, names, topics = topics)
    vis_states(bags, names, topics = topics)
    
def bug_fixing():
    dir = "/home/lucas/bags/gtsam_fusion/"
    
    identifier = ["original.", 
                  #"no_corruption_with_update",
                  #"with_corruption_with_update",
                 # "debug_prediction_cov_0.01_init",
                 # "b395590",
                 # "89ee811",
                 #"cov0.001.",
                 "relative4"
                 ]
             
    topics = ['/kolibri/mav_state_estimator/optimization',
              '/Odometry',
              #'/debug/odom'
              ]
    topic_names = [": Pose Graph",
                 ": LIO update",
                 ": LIO debug prediction"]        

        
    files = bag_loader.find_files(dir, identifier)
    bags = bag_loader.load_bags(files, topics)
    names = [os.path.basename(file)[:-4] for file in files]
    
    for bag in bags:
        bag.reset_time(1662477000)
        #print(bag.loaded_topics)
        if not "original" in bag.path:
            bag.topic_dict.pop('/kolibri/mav_state_estimator/optimization')
            
    #for odom in bags[1].get_msgs("debug/odom")[110:]:
        #R_WB = vrpn_odom.rot_matrix.as_matrix()
        #new_mat = mean_rot_offset_WB.as_matrix().T.dot(R_WB)
        #new_rot = R.from_matrix(new_mat) # transpose for 
        #vrpn_odom.euler = new_rot.as_euler('ZYX', degrees=True) # before XY  aligned!!
        
        #old implementation:
        #vrpn_odom.euler = vrpn_odom.rot_matrix.as_euler('ZYX', degrees=True) # before XY  aligned!!
        # this means we have the classic eueler angles (XYZ not ZXY)!
        
        
        #pass
        
    #vis_quat(bags, names, topics = topics)
    vis_odom(bags, names, topics = topics)
    vis_states(bags, names, topics = topics)


if __name__ == "__main__":
    main()