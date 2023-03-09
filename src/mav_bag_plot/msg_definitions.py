import numpy as np
from scipy.spatial.transform import Rotation as R

# base class
class State():
    def __init__(self, transl = None , stamp = None):
        self.stamp = stamp
        if transl is not None:
            self.t = np.asarray([[transl.x, transl.y, transl.z]]).T
        else:
            self.t = None
        self.vel = None
        self.lin_acc = None

        self.rot_matrix = None
        self.euler = None
        
        self.rot_vel = None
        
        self.flow = None
        
    def generateOrientations(self):
        try:
            self.rot_matrix = R.from_quat(self.quat)
            self.euler = self.rot_matrix.as_euler('ZYX', degrees=True)
        except ValueError:
            pass
            
    def reset_stamp(self, stamp):
        self.stamp = self.stamp - stamp

# inherited msgs from base
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
        self.quat = [quat.x, quat.y, quat.z, quat.w]
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
        super().__init__(stamp = stamp)
        
        self.quat = [quat.x, quat.y, quat.z, quat.w]
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

class OpticalFlow(State):
    def __init__(self, flow, rot_integral, range, integration_interval, stamp = None):
        self.stamp = stamp

        # sensor frame
        v_x_loc_uncorrected = flow[0] * range / integration_interval # this is the absolute change?
        v_y_loc_uncorrected = flow[1] * range / integration_interval

        v_x_loc = (flow[0] - rot_integral[0]) * range / integration_interval # this is the absolute change?
        v_y_loc = (flow[1] - rot_integral[1]) * range / integration_interval

        #writing it down in body frame
        self.t = np.asarray([[0.0, 0.0, range]]).T
        self.vel = PseudoVec(-v_y_loc, 
                             -v_x_loc, 
                             0)

        self.lin_acc = None
        self.flow = [-flow[1], -flow[0]]
        self.integration_interval = integration_interval

        self.range = range
        self.rot_vel = PseudoVec(-rot_integral[1] / integration_interval,
                                 -rot_integral[0] / integration_interval,
                                0.0)
        self.rot_integral = rot_integral


        self.vel_uncorrected= PseudoVec(-v_y_loc_uncorrected, 
                                        -v_x_loc_uncorrected, 
                                        0)


        self.euler = None

    def transform_Flow(self):
        print("implement transform rot_vel")
        pass


## Vector surrogate for manipulating data
class PseudoVec():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z