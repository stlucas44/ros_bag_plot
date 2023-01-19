import numpy as np
from scipy.spatial.transform import Rotation as R


class State():
    def __init__(self, transl, stamp = None):
        self.stamp = stamp
        self.t = np.asarray([[transl.x, transl.y, transl.z]]).T
        
        self.vel = None
        self.rot_matrix = None
        self.euler = None
        
        self.rot_vel = None
        
    def generateOrientations(self):
        try:
            self.rot_matrix = R.from_quat([self.quat.x, self.quat.y, self.quat.z, self.quat.w])
            self.euler = self.rot_matrix.as_euler('zxy', degrees=True)
        except ValueError:
            pass
            
    def reset_stamp(self, stamp):
        self.stamp = self.stamp - stamp

class TF(State):
    def __init__(self, transl, quat, stamp = None):
        super().__init__(transl, stamp)
        
        self.quat = quat
        self.generateOrientations()

    def transformPoint(self, point):
        return np.dot(self.rot_matrix.as_matrix(), point) + self.t
        
class Odom(State):
    def __init__(self, transl, quat, vel, rot_vel, stamp = None):
        super().__init__(transl, stamp)
        self.quat = quat
        self.vel = vel
        self.rot_vel = rot_vel

        self.rot_matrix = None
        self.euler = None
        
        self.generateOrientations()

    def transformPoint(self, point):
        #print(self.rot_matrix.as_matrix(), "\n", self.t)
        return np.dot(self.rot_matrix.as_matrix(), point) + self.t

class Point(State):
    # default constructor applied
    def generateOrientations(self):
        raise Exception("Can't generate rot_mat from point")
        pass

    def transformPoint(self, point):
        #print(self.rot_matrix.as_matrix(), "\n", self.t)
        if(rot_mat is None):
            raise Exception("No rotation matrix found for imu msg")
        return np.dot(self.rot_matrix.as_matrix(), point) + self.t

class Imu(State):
    def __init__(self, quat, rot_vel, lin_acc, stamp = None):
        self.stamp = stamp
        
        self.t = None
        
        self.quat = quat
        self.rot_matrix = None
        self.euler = None
        
        self.vel = None
        self.rot_vel = rot_vel
        
        self.lin_acc = lin_acc
        self.generateOrientations()
        
    def transformPoint(self, point):
        #print(self.rot_matrix.as_matrix(), "\n", self.t)
        if(rot_mat is None):
            raise Exception("No rotation matrix found for imu msg")
        return np.dot(self.rot_matrix.as_matrix(), point) + self.t