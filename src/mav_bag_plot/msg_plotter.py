import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import scipy
from scipy.spatial.transform import Rotation as R
import open3d as o3d

###
### main functions to plot states, odometries, velocities and imu data
###

def vis_states(bags, names, topics = ['/Odometry'], topic_names = None):
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(211)# projection='3d')
    ax1 = fig.add_subplot(212)
    
    if topic_names is None:
        topic_names = topic_names
    
    stamp_counter = 0
    for bag, name in zip(bags, names):
        for topic, topic_name in zip(topics, topic_names):
            msgs = bag.get_msgs(topic)
            plot_state_estimate_2D(msgs, ax, name + topic_name)
            plot_time_stamps(msgs, ax1, stamp_counter, name + topic_name)
            stamp_counter = stamp_counter + 1
             
    ax1.legend(markerscale=3.)
    #fig.tight_layout()
    #ax.title("Matching timestamps (but not receipt time!!)")
    plt.show()

def vis_odom(bags, names, topics = ['/Odometry'], topic_names = None):
    # create plot
    #fig1 = plt.figure(figsize=(8, 8))
    #fig1_ax1 = plt.subplot2grid(shape=(3, 3), loc=(0, 0), colspan=2, rowspan=3)
    #fig1_ax2 = plt.subplot2grid(shape=(3, 3), loc=(0, 2))
    #fig1_ax3 = plt.subplot2grid(shape=(3, 3), loc=(1, 2))
    #fig1_ax4 = plt.subplot2grid(shape=(3, 3), loc=(2, 2))
    
    fig2 = plt.figure(figsize=(8, 8))
    fig2_ax1 = plt.subplot2grid(shape=(6, 1), loc=(0, 0))
    fig2_ax2 = plt.subplot2grid(shape=(6, 1), loc=(1, 0))
    fig2_ax3 = plt.subplot2grid(shape=(6, 1), loc=(2, 0))
    fig2_ax4 = plt.subplot2grid(shape=(6, 1), loc=(3, 0))
    fig2_ax5 = plt.subplot2grid(shape=(6, 1), loc=(4, 0))
    fig2_ax6 = plt.subplot2grid(shape=(6, 1), loc=(5, 0))
    
    if topic_names is None:
        topic_names = topics
    
    for bag, name in zip(bags, names):
        for topic, topic_name in zip(topics, topic_names):
            msgs = bag.get_msgs(topic)
            
            #plot_state_estimate_2D(odoms, fig1_ax1, name + ': ' + topic_name)
            #plot_orientations(odoms, [fig1_ax2, fig1_ax3, fig1_ax4], name + ': ' + topic_name)
            
            plot_state_estimate_1D(msgs, [fig2_ax1, fig2_ax2, fig2_ax3], name + ': ' + topic_name)
            plot_orientations(msgs, [fig2_ax4, fig2_ax5, fig2_ax6], name + ': ' + topic_name)

    
    #fig1_ax1.legend(markerscale=3.0)
    #fig1.tight_layout()

    plt.show()
    
def vis_vel(bags, names, topics = ['/Odometry'], topic_names = None):
    # create plot
    fig2 = plt.figure(figsize=(8, 8))
    fig2_ax1 = plt.subplot2grid(shape=(6, 1), loc=(0, 0))
    fig2_ax2 = plt.subplot2grid(shape=(6, 1), loc=(1, 0))
    fig2_ax3 = plt.subplot2grid(shape=(6, 1), loc=(2, 0))
    fig2_ax4 = plt.subplot2grid(shape=(6, 1), loc=(3, 0))
    fig2_ax5 = plt.subplot2grid(shape=(6, 1), loc=(4, 0))
    fig2_ax6 = plt.subplot2grid(shape=(6, 1), loc=(5, 0))
    
    if topic_names is None:
        topic_names = topics
    
    for bag, name in zip(bags, names):
        for topic, topic_name in zip(topics, topic_names):
            msgs = bag.get_msgs(topic)

            plot_vel(msgs, [fig2_ax1, fig2_ax2, fig2_ax3], name + ': ' + topic_name)
            plot_rot_vel(msgs, [fig2_ax4, fig2_ax5, fig2_ax6], name + ': ' + topic_name)

    
    fig2_ax1.legend(markerscale=3.0)
    #fig2.tight_layout()

    plt.show()
    
    
def vis_imu(bags, names, topics = ['imu/data_raw'], topic_names = None):
    # create plot
    fig2 = plt.figure(figsize=(8, 8))
    fig2_ax1 = plt.subplot2grid(shape=(6, 1), loc=(0, 0)) #accel x
    fig2_ax2 = plt.subplot2grid(shape=(6, 1), loc=(1, 0)) #accel y
    fig2_ax3 = plt.subplot2grid(shape=(6, 1), loc=(2, 0)) # accel z
    fig2_ax4 = plt.subplot2grid(shape=(6, 1), loc=(3, 0)) 
    fig2_ax5 = plt.subplot2grid(shape=(6, 1), loc=(4, 0))
    fig2_ax6 = plt.subplot2grid(shape=(6, 1), loc=(5, 0))
    
    if topic_names is None:
        topic_names = topics
    
    for bag, name in zip(bags, names):
        for topic, topic_name in zip(topics, topic_names):
            msgs = bag.get_msgs(topic)

            plot_accelerations(msgs, [fig2_ax1, fig2_ax2, fig2_ax3], name + ': ' + topic_name)
            plot_rot_vel(msgs, [fig2_ax4, fig2_ax5, fig2_ax6], name + ': ' + topic_name)

    
    fig2_ax1.legend(markerscale=3.0)
    #fig2.tight_layout()
    plt.show()
        
####
#### helpers for specific types
#### 

def plot_state_estimate_1D(list_of_containers, ax, label = None):
    if not container_ok(list_of_containers):
        return
        
    if not translation_ok(list_of_containers):
        return
    
    translations = np.empty((3,1))
    stamps = list()
    
    for container in list_of_containers:
        translations = np.append(translations, container.t, axis = 1)
        stamps.append(container.stamp)
    
    translations = np.delete(translations, 0, axis = 1) # delete the first row?

    ax[0].scatter(stamps, translations[0], s= 4, label=label)
    ax[0].set_title("x_transl")
    ax[0].legend(markerscale=3.)

    ax[1].scatter(stamps, translations[1], s= 4, label=label)
    ax[1].set_title("y_transl")
    ax[1].legend(markerscale=3.)

    ax[2].scatter(stamps, translations[2], s= 4, label=label)
    ax[2].set_title("z_transl")
    ax[2].legend(markerscale=3.)
    
def plot_state_estimate_2D(list_of_containers, ax, label = None):
    if not container_ok(list_of_containers):
        return
    
    if not translation_ok(list_of_containers):
        return
    
    translations = np.empty((3,1))
    for container in list_of_containers:
        translations = np.append(translations, container.t, axis = 1)
        
    ax.scatter(translations[0], translations[1], s= 4, label=label)
    ax.axis('equal')
    
def plot_state_estimate_3D(tfs, ax):
    if not container_ok(list_of_containers):
        return
    if not translation_ok(container):
        return
    
    translations = np.empty((3,1))
    
    for tf in tfs:
        translations = np.append(translations, tf.t, axis = 1);
        
    ax.scatter(translations[0], 
               translations[1],
               translations[2], 
               c='g', s= 4)
               
def plot_orientations(container, axes, label = None):
    if not container_ok(container):
        return
    if not orientation_ok(container):
        return
    
    rpy = [list(), list(), list()]
    stamps = list()
    titles = ["yaw", "pitch", "roll"]
    
    for element in container:
        for angle, sub_list in zip(element.euler, rpy):
            sub_list.append(angle)
        
        stamps.append(element.stamp)
    
    for ax, data, title in zip(axes, rpy, titles):
        ax.scatter(stamps, data, s = 4, label=label)
        ax.legend(markerscale=3.0)
        ax.set_title(title)
        
def plot_accelerations(container, axes, label = None, title = "accel"):
    if not container_ok(container):
        return
    
    acc = [list(), list(), list()]
    stamps = list()
    titles = ["x_accel", "y_accel", "z_accel"]
    
    for element in container:
        stamps.append(element.stamp)
        acc[0].append(element.lin_acc.x)
        acc[1].append(element.lin_acc.y)
        acc[2].append(element.lin_acc.z)

    
    for ax, data, title in zip(axes, acc, titles):
        ax.scatter(stamps, data, s = 4, label=label)
        ax.legend(markerscale=3.0)
        ax.set_title(title)
        ax.set_ylabel("m/s2")
        
        
def plot_vel(container, axes, label = None):
    if not container_ok(container):
        return
    if not vel_ok(container):
        return
    
    r_vel = [list(), list(), list()]
    stamps = list()
    titles = ["x_vel", "y_vel", "z_vel"]
    
    for element in container:
        stamps.append(element.stamp)
        r_vel[0].append(element.vel.x)
        r_vel[1].append(element.vel.y)
        r_vel[2].append(element.vel.z)
    
    for ax, data, title in zip(axes, r_vel, titles):
        ax.scatter(stamps, data, s = 4, label=label)
        ax.legend(markerscale=3.0)
        ax.set_ylabel("m/s")
        ax.set_title(title)
        
        
def plot_rot_vel(container, axes, label = None):
    if not container_ok(container):
        return
    
    r_vel = [list(), list(), list()]
    stamps = list()
    titles = ["x_rot_vel", "y_rot_vel", "z_rot_vel"]
    
    for element in container:
        stamps.append(element.stamp)
        r_vel[0].append(element.rot_vel.x)
        r_vel[1].append(element.rot_vel.y)
        r_vel[2].append(element.rot_vel.z)
    
    for ax, data, title in zip(axes, r_vel, titles):
        ax.scatter(stamps, data, s = 4, label=label)
        ax.legend(markerscale=3.0)
        ax.set_ylabel("rad/s")
        ax.set_title(title)
    
def plot_time_stamps(list_of_containers, ax, value = 0, label = None):
    if not container_ok(list_of_containers):
        return
        
    times = []
    initial_stamp = list_of_containers[0].stamp
    for container in list_of_containers:
        times.append(container.stamp - initial_stamp)
    ax.scatter(times, [value for t in times], s = 4, label = label)


###
### checkers to prevent 
###

def container_ok(list_of_containers):
    if list_of_containers is None or not list_of_containers: # If none or empty
        print("skipping, no msgs")
        return False
    return True

def translation_ok(list_of_containers):
    if list_of_containers[0].t is None:
        print("skipping, no translation")
        return False
    return True
        
def vel_ok(list_of_containers):
    if list_of_containers[0].vel is None:
        print("skipping, no velocity")
        return False
    return True

def orientation_ok(list_of_containers):
    if list_of_containers[0].euler is None:
        print("skipping, no velocity")
        return False
    return True
        