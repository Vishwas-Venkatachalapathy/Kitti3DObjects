import os
import sys
import pykitti
import numpy as np
import parseTrackletXML as xmlParser

class DataProcessor:
    path = ""

    def __init__(self, data_path, date, drive,calibrated=True):
        path = data_path
        dataset = pykitti.raw(data_path, date, drive)
        print(np.shape(dataset.calib))
        # dataset = pykitti.raw(basedir, date, drive)

        # Load the data
        # if calibrated:
        #     dataset.load_calib()  # Calibration data are accessible as named tuples

        np.set_printoptions(precision=4, suppress=True)
        print('\nDrive: ' + str(dataset.drive))
        print('\nFrame range: ' + str(dataset.frames))

        # if calibrated:
        #     print('\nIMU-to-Velodyne transformation:\n' + str(dataset.calib.T_velo_imu))
        #     print('\nGray stereo pair baseline [m]: ' + str(dataset.calib.b_gray))
        #     print('\nRGB stereo pair baseline [m]: ' + str(dataset.calib.b_rgb))

    def align(self):
        print()

    def getDatagen(self):
        return None


    def prepare_velo_points(swlf,pts3d_raw):
        """Replaces the reflectance value by 1, and tranposes the array, so
           points can be directly multiplied by the camera projection matrix"""

        pts3d = pts3d_raw
        # Reflectance > 0
        pts3d = pts3d[pts3d[:, 3] > 0, :]
        pts3d[:, 3] = 1
        return pts3d.transpose()

    def project_velo_points_in_img(self,pts3d, T_cam_velo, Rrect, Prect):
        """Project 3D points into 2D image. Expects pts3d as a 4xN
           numpy array. Returns the 2D projection of the points that
           are in front of the camera only an the corresponding 3D points."""

        # 3D points in camera reference frame.
        pts3d_cam = Rrect.dot(T_cam_velo.dot(pts3d))

        # Before projecting, keep only points with z>0
        # (points that are in fronto of the camera).
        idx = (pts3d_cam[2, :] >= 0)
        pts2d_cam = Prect.dot(pts3d_cam[:, idx])

        return pts3d[:, idx], pts2d_cam / pts2d_cam[2, :]
