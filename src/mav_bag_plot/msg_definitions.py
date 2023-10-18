import numpy as np
from scipy.spatial.transform import Rotation as R
import sensor_msgs.point_cloud2 as pc2

# base class
class State():
    def __init__(self, transl = None , stamp = None, receipt_time = None):
        self.stamp = stamp
        self.receipt_time = receipt_time
        if transl is not None:
            self.t = np.asarray([[transl.x, transl.y, transl.z]]).T
        else:
            self.t = None
        self.vel = None
        self.lin_acc = None

        self.quat=None
        self.rot_matrix = None
        self.euler = None
        
        self.rot_vel = None

        self.pose_covariance = None
        self.twist_covariance = None
        
        self.flow = None
        self.force = None
        self.torque = None

        self.points_local = None
        
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
    def __init__(self, transl, quat, stamp = None, receipt_time = None):
        super().__init__(transl, stamp = stamp, receipt_time = receipt_time)
        
        self.quat = [quat.x, quat.y, quat.z, quat.w]
        self.generateOrientations()

    def transformPoint(self, point):
        return np.dot(self.rot_matrix.as_matrix(), point) + self.t

class Pose (State):
    def __init__(self, transl, quat, pose_cov = None, stamp = None, receipt_time = None):
        super().__init__(transl, stamp = stamp, receipt_time = receipt_time)
        self.quat = [quat.x, quat.y, quat.z, quat.w]

        self.rot_matrix = None
        self.euler = None

        self.pose_covariance = np.asarray(pose_cov).reshape((6,6))

        self.generateOrientations()

    def transformPoint(self, point):
        #print(self.rot_matrix.as_matrix(), "\n", self.t)
        return np.dot(self.rot_matrix.as_matrix(), point) + self.t


class Odom(State):
    def __init__(self, transl, quat, vel, rot_vel, pose_cov = None, twist_cov = None, stamp = None, receipt_time = None):
        super().__init__(transl, stamp = stamp, receipt_time = receipt_time)
        self.quat = [quat.x, quat.y, quat.z, quat.w]
        self.vel = [vel.x, vel.y, vel.z]
        self.rot_vel = [rot_vel.x, rot_vel.y, rot_vel.z]

        self.rot_matrix = None
        self.euler = None

        self.pose_covariance = np.asarray(pose_cov).reshape((6,6))
        self.twist_covariance = np.asarray(twist_cov).reshape((6,6))

        self.generateOrientations()

    def transformPoint(self, point):
        #print(self.rot_matrix.as_matrix(), "\n", self.t)
        return np.dot(self.rot_matrix.as_matrix(), point) + self.t

class MultiDOFJointTrajectory(State):
    def __init__(self, tpoints, stamp = None, receipt_time = None):
        super().__init__(tpoints[0].transforms[0].translation, stamp = stamp, receipt_time = receipt_time)

        quat = tpoints[0].transforms[0].rotation
        self.quat = [quat.x, quat.y, quat.z, quat.w]

        self.vel = [tpoints[0].velocities[0].linear.x,
                    tpoints[0].velocities[0].linear.y,
                    tpoints[0].velocities[0].linear.z]
        self.rot_vel = [tpoints[0].velocities[0].angular.x,
                        tpoints[0].velocities[0].angular.y,
                        tpoints[0].velocities[0].angular.z]
        self.lin_acc = [tpoints[0].accelerations[0].linear.x,
                        tpoints[0].accelerations[0].linear.y,
                        tpoints[0].accelerations[0].linear.z]

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

class PointCloud2(State):
    def __init__(self, ros_msg, stamp = None, receipt_time = None):
        super().__init__(None, stamp = stamp, receipt_time = receipt_time)

        #self.quat = quat
        self.points_local = []
        self.Body2LidarTF = []
        self.Map2BaseLinkTF = []

        if ros_msg is None: # checking if constructor is trigger by LaserScan
            return

        for p in pc2.read_points(ros_msg, field_names=("x", "y", "z"), skip_nans=True):
            self.points_local.append(p)

        for point in self.points_local:
            # transform to global frame?
            # TODO: create TF2 server to transform translator to extract poses
            pass
    def transformPoint(self, point):
        pass

class LaserScan(PointCloud2):
    def __init__(self, ros_msg, stamp=None, receipt_time = None):
        super().__init__(None, stamp = stamp, receipt_time = receipt_time)

        for i, (range_measurement, intensity) in enumerate(zip(ros_msg.ranges, ros_msg.intensities)):
            angle = ros_msg.angle_min + (i * ros_msg.angle_increment)
            x,y = (range_measurement * np.cos(angle), range_measurement * np.sin(angle))
            # reject if outside range
            if x**2 + y**2 > ros_msg.range_max**2:
                continue
            self.points_local.append([x, y, intensity])

class Imu(State):
    def __init__(self, quat, rot_vel, lin_acc, stamp = None, receipt_time = None):
        super().__init__(stamp = stamp, receipt_time = receipt_time)

        self.quat = [quat.x, quat.y, quat.z, quat.w]
        self.rot_matrix = None
        self.euler = None

        self.vel = None
        self.rot_vel = [rot_vel.x, rot_vel.y, rot_vel.z]

        self.lin_acc = [lin_acc.x, lin_acc.y, lin_acc.z]
        self.generateOrientations()

    def transformPoint(self, point):
        #print(self.rot_matrix.as_matrix(), "\n", self.t)
        if(self.rot_matrix is None):
            raise Exception("No rotation matrix found for imu msg")
        return np.dot(self.rot_matrix.as_matrix(), point) + self.t

class OpticalFlow(State):
    def __init__(self, flow, rot_integral, range, integration_interval, stamp = None, receipt_time = None):
        super().__init__(stamp = stamp, receipt_time = receipt_time)

        # sensor frame
        v_x_loc_uncorrected = flow[0] * range / integration_interval # this is the absolute change?
        v_y_loc_uncorrected = flow[1] * range / integration_interval

        # attention to signs!!
        v_x_loc = (flow[0] - rot_integral[1]) * range / integration_interval # this is the absolute change?
        v_y_loc = (flow[1] + rot_integral[0]) * range / integration_interval

        #writing it down in body frame
        self.t = np.asarray([[0.0, 0.0, range]]).T
        self.vel = [-v_y_loc,
                             -v_x_loc,
                             0]

        self.lin_acc = None
        self.flow = [-flow[1], -flow[0]]
        self.integration_interval = integration_interval

        self.range = range
        self.rot_vel = [-rot_integral[1] / integration_interval,
                                 -rot_integral[0] / integration_interval,
                                0.0]
        self.rot_integral = rot_integral


        self.vel_uncorrected= [-v_y_loc_uncorrected,
                                        -v_x_loc_uncorrected,
                                        0]


        self.euler = None

    def transform_flow(self):
        print("implement transform rot_vel")
        pass


class Wrench(State):
    def __init__(self, force, torque, stamp = None, receipt_time = None):
        super().__init__(stamp = stamp, receipt_time = receipt_time)
        self.force = [force.x, force.y, force.z]
        self.torque = [torque.x, torque.y, torque.z]

class GenericStamp(State):
    def __init__(self, stamp = None, receipt_time = None):
        super().__init__(stamp = stamp, receipt_time = receipt_time)

## Vector surrogate for manipulating data
class PseudoVec():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z