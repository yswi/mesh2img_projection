#!/usr/bin/env python
import copy
import cv2, json, os
import numpy as np
from utility.tf_utility import BuildMatrix
import open3d as o3d


#### TODO ####
PATH_IMAGE = ['spatula/wok_0.jpg'] ##: your image
PATH_MESH = [ 'data/pcd/spatula/wok_state_est_0.ply'] ## TODO : A Mesh you want to project
PATH_CAMERA_CONFIG = 'config/realsense_param.json'

t = [0.9909086902827259,  0.33953991991318344, 0.15068290873272344]
q = [0.488,-0.4678, -0.49478,  0.5567567504384794]
PIXEL = 0.001 # change this to 1 depends on your mesh unit [m] or [mm]



##############
class overlaid_image():
    def __init__(self, vis = False):

        self.camera_info = {}
        self.rgb = None
        self.mesh = None
        self.depth_image = None
        self.combined_rgb = None
        self.transform = False

        self.rgb_resolution = None

        self.vis = vis
    def assign_camera_param(self, path_config):
        f = open(path_config)
        data = json.load(f)
        f.close()


        P = np.array(data["P"]).reshape(3,4)
        K = np.array(data["K"]).reshape(3,3)
        D = np.array(data["D"])
        self.camera_info = {"P": P, "K": K, "D": D}


    def read_image(self, path_img):
        print(path_img)
        self.rgb = cv2.imread(path_img)
        h, w, _ = self.rgb.shape
        self.rgb_resolution = (h, w)


    def read_mesh(self, path_mesh):
        self.mesh = o3d.io.read_triangle_mesh(path_mesh)


    def transform_mesh(self, t, q):
        '''
        :param t: Translation from source to Target frame
        :param q: Rotation from source to Target frame
        :param pcd: open3d pointcloud from Source frame
        :return: array of pointcloud
        '''
        self.transform = True
        tf_mat = BuildMatrix(t, q)
        # self.mesh.scale(1/1000, (0,0,0))
        mesh_ = self.mesh

        mesh_  = self.mesh.transform( np.linalg.inv(tf_mat))

        self.transformed_pcd = np.array(mesh_.vertices)
        print(self.transformed_pcd)
        frame_ori = o3d.geometry.TriangleMesh.create_coordinate_frame()


        frame = copy.deepcopy(frame_ori).transform(tf_mat)
        # frame_ori = o3d.geometry.TriangleMesh.create_coordinate_frame()
        if self.vis:

            o3d.visualization.draw_geometries([mesh_, frame])


    def project_mesh(self):
        pcd_array = self.transformed_pcd

        # Read RS camera parameters
        rvec = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]) #self.camera_info['P'][:3, :3]
        tvec = np.array([0., 0., 0.]) #self.camera_info['P'][:3, 3]

        camera_mat =self.camera_info['K']
        dist_coeff = self.camera_info['D']

        # Project 3D points to 2D image in Realsense
        imgpts, jac = cv2.projectPoints(pcd_array, rvec, tvec, camera_mat, dist_coeff)
        imgpts = imgpts.squeeze()

        if self.vis:
            import matplotlib.pyplot as plt
            plt.scatter(imgpts[:,0], imgpts[:,1])
            plt.show()


        imgpts = np.rint(imgpts/PIXEL).astype(int)
        print(imgpts)
        imgpts[:, 0] = np.clip(imgpts[:,0], 0, self.rgb_resolution[1]-1)
        imgpts[:, 1] = np.clip(imgpts[:, 1], 1, self.rgb_resolution[0]-1)

        # overlay 2D image with the existing rgb image
        self._pcd_2_depth(imgpts)
        return imgpts  # 2D array





    ## PCD -> 2D image. Input : VIRDO pointcloud
    def init_latest_projected_pcd(self):
        h, w, _ = self.rgb.shape

        print("realsense resolution", h, w)
        # img_depth = np.zeros((h, w, 3))
        img_depth = copy.deepcopy(self.rgb)
        self.projected_pcd = img_depth


    def _pcd_2_depth(self, imgpts):
        self.init_latest_projected_pcd()
        pcd_projected = self.projected_pcd

        for dx, dy in imgpts:
            self.projected_pcd[dy-4, dx+6, :] = [255, 0, 255]  # Paint Reconstruction in magenta
            # self.projected_pcd[dy+2, dx-6, :] = [255, 0, 255]  # Paint Reconstruction in magenta
            # self.projected_pcd[ dy+20, dx, :] = [255, 0, 255] # Paint Reconstruction in magenta
        if self.vis:
            cv2.imshow("empty", self.projected_pcd)
            cv2.waitKey(0)
        print(pcd_projected.shape)


    def overlay_pcd_on_raw(self):
        src1 = self.rgb # realsense image
        src2 = self.projected_pcd

        # blend two images
        alpha = 0.8
        beta = (1.0 - alpha)
        self.combined_rgb = cv2.addWeighted(src1, alpha, src2, beta, 0.0)

    def save_combined_rgb(self, filename):
        cv2.imwrite(filename, self.projected_pcd)


for image_idx, path_image in enumerate(PATH_IMAGE):

    path_mesh = os.path.join(PATH_MESH[image_idx])

    oli = overlaid_image()
    oli.assign_camera_param(PATH_CAMERA_CONFIG)
    oli.read_image(path_image)

    oli.read_mesh(path_mesh)
    oli.transform_mesh(t, q)
    oli.project_mesh()
    oli.overlay_pcd_on_raw()

    PATH_SAVE_RESULT = f'result/{path_image_}'
    print(PATH_SAVE_RESULT)
    oli.save_combined_rgb(PATH_SAVE_RESULT)
