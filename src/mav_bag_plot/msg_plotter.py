import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import scipy
from scipy.fft import fft, fftfreq
from scipy.spatial.transform import Rotation as R
#import open3d as o3d

###
### main functions to plot states, odometries, velocities and imu data
###

def vis_states(bags, names, topics = ['/Odometry'], topic_names = None):
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(211)# projection='3d')
    ax1 = fig.add_subplot(212)
    
    if topic_names is None:
        topic_names = topics
    
    stamp_counter = 0
    for bag, name in zip(bags, names):
        for topic, topic_name in zip(topics, topic_names):
            msgs = bag.get_msgs(topic)
            plot_state_estimate_2D(msgs, ax, name + topic_name)
            plot_time_stamps(msgs, ax1, stamp_counter, name + topic_name)
            stamp_counter = stamp_counter + 1

    ax.legend(markerscale=3.0, loc=2)
    ax1.legend(markerscale=3.0, loc= 2 )
    #fig.tight_layout()
    #ax.title("Matching timestamps (but not receipt time!!)")
    plt.draw()

    return ax, ax1

def vis_pcl_local(bags, names, stamp, topics=['/pointcloud_2d'], topic_names=None):
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111)  # projection='3d')
    #ax1 = fig.add_subplot(111)

    if topic_names is None:
        topic_names = topics

    # TODO: introduce slider for pcl selection
    # TODO: introduce play button for replay

    stamp_counter = 0
    for bag, name in zip(bags, names):
        for topic, topic_name in zip(topics, topic_names):
            msgs = bag.get_msgs(topic)
            plot_pcl_2d_local(msgs, ax, stamp, name + topic_name)
            #plot_time_stamps(msgs, ax1, stamp_counter, name + topic_name)
            stamp_counter = stamp_counter + 1

    ax.legend(markerscale=3.0, loc=2)
    # fig.tight_layout()
    #ax.title("Matching timestamps (but not receipt time!!)")
    ax.arrow(0,0, 0.5, 0, color='k')
    plt.draw()

def vis_odom(bags, names, topics = ['/Odometry'], topic_names = None):
    # create plot
    #fig1 = plt.figure(figsize=(8, 8))
    #fig1_ax1 = plt.subplot2grid(shape=(3, 3), loc=(0, 0), colspan=2, rowspan=3)
    #fig1_ax2 = plt.subplot2grid(shape=(3, 3), loc=(0, 2))
    #fig1_ax3 = plt.subplot2grid(shape=(3, 3), loc=(1, 2))
    #fig1_ax4 = plt.subplot2grid(shape=(3, 3), loc=(2, 2))
    
    fig2 = plt.figure(figsize=(16, 9))
    fig2_ax1 = plt.subplot2grid(shape=(6, 1), loc=(0, 0))
    fig2_ax2 = plt.subplot2grid(shape=(6, 1), loc=(1, 0), sharex = fig2_ax1)
    fig2_ax3 = plt.subplot2grid(shape=(6, 1), loc=(2, 0), sharex = fig2_ax1)
    fig2_ax4 = plt.subplot2grid(shape=(6, 1), loc=(3, 0), sharex = fig2_ax1)
    fig2_ax5 = plt.subplot2grid(shape=(6, 1), loc=(4, 0), sharex = fig2_ax1)
    fig2_ax6 = plt.subplot2grid(shape=(6, 1), loc=(5, 0), sharex = fig2_ax1)
    
    if topic_names is None:
        topic_names = topics
    
    for bag, name in zip(bags, names):
        for topic, topic_name in zip(topics, topic_names):
            msgs = bag.get_msgs(topic)
            
            #plot_state_estimate_2D(odoms, fig1_ax1, name + ': ' + topic_name)
            #plot_orientations(odoms, [fig1_ax2, fig1_ax3, fig1_ax4], name + ': ' + topic_name)
            
            plot_state_estimate_1D(msgs, [fig2_ax1, fig2_ax2, fig2_ax3], name + ': ' + topic_name)
            plot_orientations(msgs, [fig2_ax4, fig2_ax5, fig2_ax6], name + ': ' + topic_name)
            
    
    #fig1_ax1.legend(markerscale=3.0, loc= 2)
    fig2.tight_layout()

    plt.draw()

def vis_odom_state_w_cov(bags, names, topics = ['/Odometry'], topic_names = None):
    # create plot
    #fig1 = plt.figure(figsize=(8, 8))
    #fig1_ax1 = plt.subplot2grid(shape=(3, 3), loc=(0, 0), colspan=2, rowspan=3)
    #fig1_ax2 = plt.subplot2grid(shape=(3, 3), loc=(0, 2))
    #fig1_ax3 = plt.subplot2grid(shape=(3, 3), loc=(1, 2))
    #fig1_ax4 = plt.subplot2grid(shape=(3, 3), loc=(2, 2))
    
    fig2 = plt.figure(figsize=(16, 9))
    fig2_ax1 = plt.subplot2grid(shape=(6, 1), loc=(0, 0))
    fig2_ax2 = plt.subplot2grid(shape=(6, 1), loc=(1, 0), sharex = fig2_ax1)
    fig2_ax3 = plt.subplot2grid(shape=(6, 1), loc=(2, 0), sharex = fig2_ax1)
    fig2_ax4 = plt.subplot2grid(shape=(6, 1), loc=(3, 0), sharex = fig2_ax1)
    fig2_ax5 = plt.subplot2grid(shape=(6, 1), loc=(4, 0), sharex = fig2_ax1)
    fig2_ax6 = plt.subplot2grid(shape=(6, 1), loc=(5, 0), sharex = fig2_ax1)
    
    if topic_names is None:
        topic_names = topics
    
    for bag, name in zip(bags, names):
        for topic, topic_name in zip(topics, topic_names):
            msgs = bag.get_msgs(topic)
            
            #plot_state_estimate_2D(odoms, fig1_ax1, name + ': ' + topic_name)
            #plot_orientations(odoms, [fig1_ax2, fig1_ax3, fig1_ax4], name + ': ' + topic_name)
            
            plot_state_estimate_1D(msgs, [fig2_ax1, fig2_ax2, fig2_ax3], name + ': ' + topic_name, plot_cov=True)
            plot_orientations(msgs, [fig2_ax4, fig2_ax5, fig2_ax6], name + ': ' + topic_name)
            
    
    #fig1_ax1.legend(markerscale=3.0, loc= 2)
    fig2.tight_layout()

    plt.draw()
    
def vis_vel(bags, names, topics = ['/Odometry'], topic_names = None):
    # create plot
    fig2 = plt.figure(figsize=(16, 9))
    fig2_ax1 = plt.subplot2grid(shape=(6, 1), loc=(0, 0))
    fig2_ax2 = plt.subplot2grid(shape=(6, 1), loc=(1, 0), sharex = fig2_ax1)
    fig2_ax3 = plt.subplot2grid(shape=(6, 1), loc=(2, 0), sharex = fig2_ax1)
    fig2_ax4 = plt.subplot2grid(shape=(6, 1), loc=(3, 0), sharex = fig2_ax1)
    fig2_ax5 = plt.subplot2grid(shape=(6, 1), loc=(4, 0), sharex = fig2_ax1)
    fig2_ax6 = plt.subplot2grid(shape=(6, 1), loc=(5, 0), sharex = fig2_ax1)
    
    if topic_names is None:
        topic_names = topics
    
    for bag, name in zip(bags, names):
        for topic, topic_name in zip(topics, topic_names):
            msgs = bag.get_msgs(topic)

            plot_vel(msgs, [fig2_ax1, fig2_ax2, fig2_ax3], name + ': ' + topic_name)
            plot_rot_vel(msgs, [fig2_ax4, fig2_ax5, fig2_ax6], name + ': ' + topic_name)

    
    fig2_ax1.legend(markerscale=3.0, loc= 2)
    #fig2.tight_layout()

    plt.draw()

def vis_vel_w_cov(bags, names, topics = ['/Odometry'], topic_names = None, plot_covs = None):
    # create plot
    fig2 = plt.figure(figsize=(16, 9))
    fig2_ax1 = plt.subplot2grid(shape=(6, 1), loc=(0, 0))
    fig2_ax2 = plt.subplot2grid(shape=(6, 1), loc=(1, 0), sharex = fig2_ax1)
    fig2_ax3 = plt.subplot2grid(shape=(6, 1), loc=(2, 0), sharex = fig2_ax1)
    fig2_ax4 = plt.subplot2grid(shape=(6, 1), loc=(3, 0), sharex = fig2_ax1)
    fig2_ax5 = plt.subplot2grid(shape=(6, 1), loc=(4, 0), sharex = fig2_ax1)
    fig2_ax6 = plt.subplot2grid(shape=(6, 1), loc=(5, 0), sharex = fig2_ax1)
    
    if topic_names is None:
        topic_names = topics
    
    for bag, name in zip(bags, names):
        for topic, topic_name in zip(topics, topic_names):
            msgs = bag.get_msgs(topic)

            plot_vel(msgs, [fig2_ax1, fig2_ax2, fig2_ax3], name + ': ' + topic_name, plot_cov = True)
            plot_rot_vel(msgs, [fig2_ax4, fig2_ax5, fig2_ax6], name + ': ' + topic_name)

    
    fig2_ax1.legend(markerscale=3.0, loc= 2)
    #fig2.tight_layout()

    plt.draw()
    
    
def vis_imu(bags, names, topics = ['imu/data_raw'], topic_names = None):
    # create plot
    fig2 = plt.figure(figsize=(8, 8))
    fig2_ax1 = plt.subplot2grid(shape=(6, 1), loc=(0, 0)) #accel x
    fig2_ax2 = plt.subplot2grid(shape=(6, 1), loc=(1, 0), sharex = fig2_ax1) #accel y
    fig2_ax3 = plt.subplot2grid(shape=(6, 1), loc=(2, 0), sharex = fig2_ax1) # accel z
    fig2_ax4 = plt.subplot2grid(shape=(6, 1), loc=(3, 0), sharex = fig2_ax1) 
    fig2_ax5 = plt.subplot2grid(shape=(6, 1), loc=(4, 0), sharex = fig2_ax1)
    fig2_ax6 = plt.subplot2grid(shape=(6, 1), loc=(5, 0), sharex = fig2_ax1)
    
    if topic_names is None:
        topic_names = topics
    
    for bag, name in zip(bags, names):
        for topic, topic_name in zip(topics, topic_names):
            msgs = bag.get_msgs(topic)
            plot_accelerations(msgs, [fig2_ax1, fig2_ax2, fig2_ax3], name + ': ' + topic_name)
            plot_rot_vel(msgs, [fig2_ax4, fig2_ax5, fig2_ax6], name + ': ' + topic_name)

    
    fig2_ax1.legend(markerscale=3.0, loc= 2)
    #fig2.tight_layout()
    plt.draw()
    
def vis_fft(bags, names, topics = ['imu/data_raw'], topic_names = None):
    # create plot
    fig2 = plt.figure(figsize=(8, 8))
    fig2_ax1 = plt.subplot2grid(shape=(6, 1), loc=(0, 0)) #accel x
    fig2_ax2 = plt.subplot2grid(shape=(6, 1), loc=(1, 0), sharex = fig2_ax1) #accel y
    fig2_ax3 = plt.subplot2grid(shape=(6, 1), loc=(2, 0), sharex = fig2_ax1) # accel z
    fig2_ax4 = plt.subplot2grid(shape=(6, 1), loc=(3, 0), sharex = fig2_ax1) 
    fig2_ax5 = plt.subplot2grid(shape=(6, 1), loc=(4, 0), sharex = fig2_ax1)
    fig2_ax6 = plt.subplot2grid(shape=(6, 1), loc=(5, 0), sharex = fig2_ax1)
    
    if topic_names is None:
        topic_names = topics
    
    for bag, name in zip(bags, names):
        for topic, topic_name in zip(topics, topic_names):
            msgs = bag.get_msgs(topic)
            plot_fft(msgs, [fig2_ax1, fig2_ax2, fig2_ax3], 
                name + ': ' + topic_name, class_member="lin_acc", sampling_frequency = 200)
            plot_fft(msgs, [fig2_ax4, fig2_ax5, fig2_ax6], 
                name + ': ' + topic_name, class_member="rot_vel")
    
            
    fig2_ax1.legend(markerscale=3.0, loc= 2)
    #fig2.tight_layout()
    plt.draw()
    
    
def vis_quat(bags, names, topics = ['imu/data_raw'], topic_names = None):
    # create plot
    fig2 = plt.figure(figsize=(8, 8))
    fig2_ax1 = plt.subplot2grid(shape=(6, 1), loc=(0, 0)) #accel x
    fig2_ax2 = plt.subplot2grid(shape=(6, 1), loc=(1, 0), sharex = fig2_ax1) #accel y
    fig2_ax3 = plt.subplot2grid(shape=(6, 1), loc=(2, 0), sharex = fig2_ax1) # accel z
    fig2_ax4 = plt.subplot2grid(shape=(6, 1), loc=(3, 0), sharex = fig2_ax1) 
    #fig2_ax5 = plt.subplot2grid(shape=(6, 1), loc=(4, 0), sharex = fig2_ax1)
    #fig2_ax6 = plt.subplot2grid(shape=(6, 1), loc=(5, 0), sharex = fig2_ax1)
    
    if topic_names is None:
        topic_names = topics
    
    for bag, name in zip(bags, names):
        for topic, topic_name in zip(topics, topic_names):
            msgs = bag.get_msgs(topic)
            plot_quat(msgs, [fig2_ax1, fig2_ax2, fig2_ax3, fig2_ax4], 
                name + ': ' + topic_name)
    
            
    fig2_ax1.legend(markerscale=3.0, loc= 2)
    #fig2.tight_layout()
    plt.draw()
        
def vis_flow(bags, names, topics = ['/stork/optical_flow125'], topic_names = None):
    # create plot
    fig2 = plt.figure(figsize=(8, 8))
    fig2_ax1 = plt.subplot2grid(shape=(6, 1), loc=(0, 0))
    fig2_ax2 = plt.subplot2grid(shape=(6, 1), loc=(1, 0), sharex = fig2_ax1)
    fig2_ax3 = plt.subplot2grid(shape=(6, 1), loc=(2, 0), sharex = fig2_ax1)
    fig2_ax4 = plt.subplot2grid(shape=(6, 1), loc=(3, 0), sharex = fig2_ax1) 
    fig2_ax5 = plt.subplot2grid(shape=(6, 1), loc=(4, 0), sharex = fig2_ax1)
    fig2_ax6 = plt.subplot2grid(shape=(6, 1), loc=(5, 0), sharex = fig2_ax1)

    if topic_names is None:
        topic_names = topics
    
    for bag, name in zip(bags, names):
        for topic, topic_name in zip(topics, topic_names):
            msgs = bag.get_msgs(topic)
            plot_flow(msgs, [fig2_ax1, fig2_ax2, fig2_ax3], 
                name + ': ' + topic_name)
            #plot_rot_vel(msgs, [fig2_ax4, fig2_ax5, fig2_ax6], name + ': ' + topic_name)
            plot_vel(msgs, [fig2_ax4, fig2_ax5, fig2_ax6], name + ': ' + topic_name)

    fig2_ax1.legend(markerscale=3.0, loc= 2)
    #fig2.tight_layout()
    plt.draw()


def vis_timing(bags, names, topics = ['/foo'], topic_names = None):
    fig = plt.figure(figsize=(8, 8))
    ax1 = fig.add_subplot(211)# projection='3d')
    ax2 = fig.add_subplot(212)
    
    if topic_names is None:
        topic_names = topics
    
    stamp_counter = 0
    for bag, name in zip(bags, names):
        for topic, topic_name in zip(topics, topic_names):
            msgs = bag.get_msgs(topic)
            plot_time_stamps(msgs, ax1, stamp_counter, name + topic_name)
            plot_time_diffs(msgs, ax2, stamp_counter, name + topic_name)

            stamp_counter = stamp_counter + 1
            
    ax1.legend(markerscale=3.0, loc= 2 )
    ax2.legend(markerscale=3.0, loc= 2 )
    #fig.tight_layout()
    #ax.title("Matching timestamps (but not receipt time!!)")
    plt.draw()


def vis_wrench(bags, names, topics = ['/stork/optical_flow125'], topic_names = None):
    # create plot
    fig2 = plt.figure(figsize=(8, 8))
    fig2_ax1 = plt.subplot2grid(shape=(6, 1), loc=(0, 0))
    fig2_ax2 = plt.subplot2grid(shape=(6, 1), loc=(1, 0), sharex = fig2_ax1)
    fig2_ax3 = plt.subplot2grid(shape=(6, 1), loc=(2, 0), sharex = fig2_ax1)
    fig2_ax4 = plt.subplot2grid(shape=(6, 1), loc=(3, 0), sharex = fig2_ax1) 
    fig2_ax5 = plt.subplot2grid(shape=(6, 1), loc=(4, 0), sharex = fig2_ax1)
    fig2_ax6 = plt.subplot2grid(shape=(6, 1), loc=(5, 0), sharex = fig2_ax1)

    if topic_names is None:
        topic_names = topics
    
    for bag, name in zip(bags, names):
        for topic, topic_name in zip(topics, topic_names):
            msgs = bag.get_msgs(topic)
            plot_force(msgs, [fig2_ax1, fig2_ax2, fig2_ax3], 
                name + ': ' + topic_name)
            #plot_rot_vel(msgs, [fig2_ax4, fig2_ax5, fig2_ax6], name + ': ' + topic_name)
            plot_torque(msgs, [fig2_ax4, fig2_ax5, fig2_ax6], name + ': ' + topic_name)

    fig2_ax1.legend(markerscale=3.0, loc= 2)
    #fig2.tight_layout()
    plt.draw()
####
#### helpers for specific types
#### 

def plot_state_estimate_1D(list_of_containers, ax, label = None, plot_cov = False):
    if not container_ok(list_of_containers):
        return
        
    if not translation_ok(list_of_containers):
        return
    
    translations = np.empty((3,1))
    covs = np.empty((3,1))
    stamps = list()
    
    for container in list_of_containers:
        translations = np.append(translations, container.t, axis = 1)
        stamps.append(container.stamp)

        if plot_cov and container.pose_covariance is not None:
            # extract the diagoals, keep the first 3
            cov_diags = np.diag(container.pose_covariance)[:3].reshape((3,1))
            covs = np.append(covs, cov_diags, axis = 1)
    
    translations = np.delete(translations, 0, axis = 1) # delete the first row?
    covs = np.delete(covs, 0, axis = 1) # delete the first row?

    names = ["x_transl", "y_transl", "z_transl"]

    if plot_cov and covs.size!=0:
        for a, t, cov, n in zip(ax, translations, covs, names):
            upper = t + np.sqrt(cov)
            lower = t - np.sqrt(cov)

            a.plot(stamps, t, 'o-', ms = 2, lw = 0.5, label = label)
            a.fill_between(stamps, lower, upper, alpha=0.2, color = a.get_lines()[-1].get_c())

            a.set_title(n)
            a.legend(loc='center left', bbox_to_anchor=(1, 0.5),markerscale=3.)

    
    else:
        for a, t, n in zip(ax, translations, names):
            #a.scatter(stamps, t, s = 4, label = label)
            a.plot(stamps, t, 'o-', ms = 2, lw = 0.5, label = label)
            a.set_title(n)
            a.legend(loc='center left', bbox_to_anchor=(1, 0.5),markerscale=3.)
    
def plot_state_estimate_2D(list_of_containers, ax, label = None, heading_spacing = -1):
    if not container_ok(list_of_containers):
        return
    
    if not translation_ok(list_of_containers):
        return
    
    translations = [[], []]
    headings = []
    times = []

    for container in list_of_containers:
        translations[0].append(container.t[0, 0])
        translations[1].append(container.t[1, 0])
        headings.append(container.euler[0])
    
    ax.scatter(translations[0], translations[1], s= 4, label=label)
    ax.plot(translations[0], translations[1], label=label)

    if heading_spacing != -1:
        length = 0.1
        for heading, x, y in list(zip(headings, translations[0], translations[1]))[::heading_spacing]:
            ax.arrow(x,y, length * np.cos(heading / 180 * np.pi), length * np.sin(heading / 180 * np.pi), color='k')
    ax.axis('equal')


def plot_pcl_2d_local(list_of_containers, ax, stamp, label = None):
    if not container_ok(list_of_containers):
        return
    if not points_local_ok(list_of_containers):
        return

    prev_stamp = list_of_containers[0]
    msg_index = 0

    # find first stamp after required stamp
    for i, c in enumerate(list_of_containers[1:]):
        if c.stamp > stamp:
            break
        msg_index += 1

    # load points according to stamp
    points = list_of_containers[msg_index].points_local
    points = np.asarray(points)

    #check if we really have points
    if len(points) == 0:
        return

    ax.scatter(points[:,0], points[:,1], s=4, label=label)
    #ax.plot(points[0], points[1], label=label)

    ax.axis('equal')

def plot_state_estimate_3D(list_of_containers, ax):
    if not container_ok(list_of_containers):
        return
    if not translation_ok(list_of_containers):
        return
    
    translations = np.empty((3,1))
    
    for tf in list_of_containers:
        translations = np.append(translations, tf.t, axis = 1); 
        
    ax.scatter(translations[0], 
               translations[1],
               translations[2], 
               c='g', s= 4)
               
def plot_orientations(container, axes, label = None):
    if not container_ok(container):
        print("for label ", label)
        return
    if not orientation_ok(container):
        return
    
    ypr = [list(), list(), list()]
    stamps = list()
    titles = ["yaw", "pitch", "roll"]
    
    for element in container:
        for angle, sub_list in zip(element.euler, ypr):
            sub_list.append(angle)
        
        stamps.append(element.stamp)
    
    for ax, data, title in zip(axes, ypr, titles):
        #ax.scatter(stamps, data, s = 4, label=label)
        ax.plot(stamps, data, 'o-', ms = 2, lw = 0.5, label=label)
        ax.legend(loc='center left', bbox_to_anchor=(1, 0.5), markerscale=3.)   
     
        ax.set_title(title)
        
        
def plot_quat(container, axes, label = None):
    if not container_ok(container):
        print("for label ", label)
        return
    if not orientation_ok(container):
        return
    
    quats = [list(), list(), list(), list()]
    stamps = list()
    titles = ["x", "y", "z", "w"]
    
    for element in container:
        quat = element.quat
        for value, sub_list in zip(quat, quats):
            sub_list.append(value)
        
        stamps.append(element.stamp)
    
    for ax, data, title in zip(axes, quats, titles):
        #ax.scatter(stamps, data, s = 4, label=label)
        ax.plot(stamps, data, 'o-', ms = 2, lw = 0.5, label=label)
        ax.legend(loc='center left', bbox_to_anchor=(1, 0.5), markerscale=3.)   
     
        ax.set_title(title)
        
def plot_accelerations(container, axes, label = None, title = "accel"):
    if not container_ok(container):
        print("for label ", label)
        return

    if not accel_ok(container):
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
        ax.plot(stamps, data, 'o-', ms = 2, lw = 0.5, label=label)
        ax.legend(markerscale=3.0, loc= 2)
        ax.set_title(title)
        ax.set_ylabel("m/s2")
        
        
def plot_vel(container, axes, label = None, plot_cov = False):
    if not container_ok(container):
        print("for label ", label)
        return
    if not vel_ok(container):
        return
    
    r_vel = [list(), list(), list()]
    stds = [list(), list(), list()]
    stamps = list()
    titles = ["x_vel", "y_vel", "z_vel"]
    
    # extract data
    for element in container:
        stamps.append(element.stamp)
        r_vel[0].append(element.vel.x)
        r_vel[1].append(element.vel.y)
        r_vel[2].append(element.vel.z)

        if element.twist_covariance is not None and plot_cov:
            stds[0].append(np.sqrt(element.twist_covariance[0,0]))
            stds[1].append(np.sqrt(element.twist_covariance[1,1]))
            stds[2].append(np.sqrt(element.twist_covariance[2,2]))

    # plot covariances if desired and available
    if plot_cov and len(stds[0]) > 0:
        for ax, data, std, title in zip(axes, r_vel, stds, titles):
            ax.plot(stamps, data, 'o-', ms = 2, lw = 0.5, label=label)
            upper = [x+y for x,y  in zip(data, std)]
            lower = [x-y for x,y  in zip(data, std)]

            ax.fill_between(stamps, lower, upper, alpha=0.2, color = ax.get_lines()[-1].get_c())            
            ax.legend(markerscale=3.0, loc= 2)
            ax.set_ylabel("m/s")
            ax.set_title(title)
    else:
        for ax, data, title in zip(axes, r_vel, titles):
            ax.plot(stamps, data, 'o-', ms = 2, lw = 0.5, label=label)
            ax.legend(markerscale=3.0, loc= 2)
            ax.set_ylabel("m/s")
            ax.set_title(title)
        
        
def plot_rot_vel(container, axes, label = None):
    if not container_ok(container):
        print("for label ", label)
        return

    if not rot_vel_ok(container):
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
        ax.plot(stamps, data, 'o-', ms = 2, lw = 0.5, label=label)
        ax.legend(markerscale=3.0, loc= 2)
        ax.set_ylabel("rad/s")
        ax.set_title(title)

def plot_flow(container, axes, label = None):
    if not container_ok(container):
        print("for label ", label)
        return

    if not flow_ok(container):
        return
    
    flow = [list(), list(), list()]
    stamps = list()
    titles = ["x_flow", "y_flow", "range"]
    

    for element in container:
        stamps.append(element.stamp)
        flow[0].append(element.flow[0])
        flow[1].append(element.flow[1])
        flow[2].append(element.range)
    
    # add absolute flow
    for ax, data, title in zip(axes, flow, titles):
        ax.plot(stamps, data, 'o-', ms = 2, lw = 0.5, label=label)
        ax.legend(markerscale=3.0, loc= 2)
        #ax.set_ylabel("")
        ax.set_title(title)

    # add rot vels for comparison

def plot_force(container,axes, label):
    if not container_ok(container):
        print("for label ", label)
        return

    if not wrench_ok(container):
        return
    
    f = [list(), list(), list()]
    stamps = list()
    titles = ["f_x", "f_y", "f_z"]
    

    for element in container:
        stamps.append(element.stamp)
        #for f_i, force in zip(f, element.force): # axis wise append
        #    f_i.append(force)
        f[0].append(element.force[0])
        f[1].append(element.force[1])
        f[2].append(element.force[2])

    for ax, data, title in zip(axes, f, titles):
        ax.plot(stamps, data, 'o-', ms = 2, lw = 0.5, label=label)
        ax.legend(markerscale=3.0, loc= 2)
        ax.set_title(title)


def plot_torque(container,axes, label):
    if not container_ok(container):
        print("for label ", label)
        return

    if not wrench_ok(container):
        return
    
    t = [list(), list(), list()]
    stamps = list()
    titles = ["t_x", "t_y", "t_z"]
    

    for element in container:
        stamps.append(element.stamp)
        t[0].append(element.torque[0])
        t[1].append(element.torque[1])
        t[2].append(element.torque[2])

    for ax, data, title in zip(axes, t, titles):
        ax.plot(stamps, data, 'o-', ms = 2, lw = 0.5, label=label)
        ax.legend(markerscale=3.0, loc= 2)
        ax.set_title(title)


    
def plot_time_stamps(list_of_containers, ax, value = 0, label = None):
    if not container_ok(list_of_containers):
        return
        
    times = []
    initial_stamp = list_of_containers[0].stamp
    for container in list_of_containers:
        
        times.append(container.stamp)# - initial_stamp)
    ax.scatter(times, [value for t in times], s = 4, label = label)

def plot_time_diffs(list_of_containers, ax, value = 0, label = None):
    if not container_ok(list_of_containers):
        return

    plot_interval = hasattr(list_of_containers[0], 'integration_interval')
        
    times = []
    diffs = []
    measurements = []

    prev = list_of_containers[0].stamp - 0.1
    for container in list_of_containers:
        dt = (container.stamp - prev) if not (container.stamp - prev) == 0  else -1
        diffs.append(1/dt)
        times.append(container.stamp)

        if plot_interval:
            measurements.append(1/  container.integration_interval)
        prev = container.stamp
    
    ax.scatter(times, diffs, s = 4, label = label + '_stamp_diff')

    # if it has an internal time measurement
    if plot_interval:
        ax.scatter(times,  measurements, s = 4, label = label + '_measured')

    
def plot_fft(container, axes, label, class_member = "lin_acc", sampling_frequency = 1/191.0):
    if not container_ok(container):
        print("for label ", label)
        return
    
    acc = [list(), list(), list()]
    stamps = list()
    titles = ["x_" + class_member, "y_" + class_member, "z_" + class_member]
    
    for element in container:
        stamps.append(element.stamp)
        vec = getattr(element, class_member)
        acc[0].append(vec.x)
        acc[1].append(vec.y)
        acc[2].append(vec.z)

    
    for ax, data, title in zip(axes, acc, titles):
        ax.scatter(*get_fft(data), alpha=0.8, s = 4, label=label)
        ax.legend(markerscale=3.0, loc= 2)
        ax.set_title(title)
        ax.set_ylabel("Magnitude")
        ax.set_xlabel("Frequency")
        #ax.set_yscale('log')
        ax.set_ylim(bottom=0.5)

def get_fft(signal, sampling_frequency = 1/191.0): # Adis on stork runs on 191 Hz
    N = len(signal)
    fft_data = fft(signal)
    fft_freq = fftfreq(len(signal), sampling_frequency)
    return fft_freq[0:N//2], ((np.abs(fft_data)))[0:N//2]

###
### checkers to prevent 
###

def container_ok(list_of_containers):
    if list_of_containers is None or not list_of_containers: # If none or empty
        print("skipping, no msgs ", end = "")
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

def accel_ok(list_of_containers):
    if list_of_containers[0].lin_acc is None:
        print("skipping, no linear acceleration")
        return False
    return True

def orientation_ok(list_of_containers):
    if list_of_containers[0].euler is None:
        print("skipping, no orientation")
        return False
    return True
        
def rot_vel_ok(list_of_containers):
    if list_of_containers[0].rot_vel is None:
        print("skipping, no rotational velocity")
        return False
    return True

def flow_ok(list_of_containers):
    if list_of_containers[0].flow is None:
        print("skipping, no flow")
        return False
    return True

def wrench_ok(list_of_containers):
    if list_of_containers[0].force is None:
        print("skipping, no force")
        return False
    return True

def points_local_ok(list_of_containers):
    if list_of_containers[0].points_local is None or len(list_of_containers[0].points_local) == 0:
        print("skipping, no points_local")
        return False
    return True