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

        self.pose_covariance = None
        self.twist_covariance = None
        
        self.flow = None
        self.force = None
        self.torque = None
        
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
    def __init__(self, transl, quat, vel, rot_vel, pose_cov = None, twist_cov = None, stamp = None):
        super().__init__(transl, stamp)
        self.quat = [quat.x, quat.y, quat.z, quat.w]
        self.vel = vel
        self.rot_vel = rot_vel

        self.rot_matrix = None
        self.euler = None

        self.pose_covariance = np.asarray(pose_cov).reshape((6,6))
        self.twist_covariance = np.asarray(twist_cov).reshape((6,6))
        
        self.generateOrientations()

    def transformPoint(self, point):
        #print(self.rot_matrix.as_matrix(), "\n", self.t)
        return np.dot(self.rot_matrix.as_matrix(), point) + self.t

class MultiDOFJointTrajectory(State):
    def __init__(self, tpoints, stamp = None):
        super().__init__(tpoints[0].transforms[0].translation, stamp = stamp)

        quat = tpoints[0].transforms[0].rotation
        self.quat = [quat.x, quat.y, quat.z, quat.w]

        self.vel = tpoints[0].velocities[0].linear
        self.rot_vel = tpoints[0].velocities[0].angular
        self.lin_acc = tpoints[0].accelerations[0].linear

        self.generateOrientations()

class Point(State):
    # default constructor applied
    def generateOrientations(self):
        raise Exception("Can't generate rot_mat from point")
        pass

    def transformPoint(self, point):
        #print(self.rot_matrix.as_matrix(), "\n", self.t)
        if(self.rot_matrix is None):
            raise Exception("No rotation matrix found for point msg")
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
        if(self.rot_matrix is None):
            raise Exception("No rotation matrix found for imu msg")
        return np.dot(self.rot_matrix.as_matrix(), point) + self.t

class OpticalFlow(State):
    def __init__(self, flow, rot_integral, range, integration_interval, stamp = None):
        super().__init__(stamp = stamp)

        # sensor frame
        v_x_loc_uncorrected = flow[0] * range / integration_interval # this is the absolute change?
        v_y_loc_uncorrected = flow[1] * range / integration_interval

        # attention to signs!!
        v_x_loc = (flow[0] - rot_integral[1]) * range / integration_interval # this is the absolute change?
        v_y_loc = (flow[1] + rot_integral[0]) * range / integration_interval
        
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


class Wrench(State):
    def __init__(self, force, torque, stamp):
        super().__init__(stamp = stamp)
        self.force = [force.x, force.y, force.z]
        self.torque = [torque.x, torque.y, torque.z]

## Vector surrogate for manipulating data
class PseudoVec():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z