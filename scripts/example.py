import os
 
import mav_bag_plot.bag_loader as bag_loader
from mav_bag_plot.msg_plotter import vis_states, vis_odom


def main():
    create_plot()
    
def create_plot():
    # specify directory and identifier to set which bags to load
    dir = "absolute_path_to_directory"
    identifier = ["unique_bag_name", 
                  "substring contained by set of bags"]

    # specify topics (and names to show in the legend)
    topics = ['/kolibri/mav_state_estimator/optimization',
              #'/Gnss',
              '/Odometry']
    topic_names = [": Pose Graph",
                 #": GNSS",
                 ": LIO"]          
    
    # search files and define bag names (currently bag name from path)
    files = bag_loader.find_files(dir, identifier)
    names = [os.path.basename(file)[:-3] for file in files]
    
    # load desired topics to a list of BagContainers
    bags = bag_loader.load_bags(files, topics)
    
    # create plots
    vis_odom(bags, names, topics = topics)
    vis_states(bags, names, topics = topics, topic_names = topic_names)

if __name__ == "__main__":
    main()