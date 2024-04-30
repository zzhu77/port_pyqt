from geometry_msgs.msg import PoseStamped
from rclpy.node import Node    
import numpy as np       
from scipy.linalg import svd         
import pyigtl
import open3d as o3d
import json
import vtk

class objectClient(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("node launch success：%s!" % name)

        self.ref_points_ct = np.array([])
        self.ref_points_ts = np.array([])
        self.tar_points_ts = np.array([])
        self.tar_points_ct = np.array([])

        # self.pointer = np.loadtxt("pointer_offset.txt")
        self.R = np.eye(3)
        self.t = np.zeros(3)
        # self.ndi_pose = PoseStamped()
        self.ndi_loc = np.zeros(3)
        self.eps_start = -1
        self.eps_count = 0
        self.if_surface = False
        self.shape_points = np.array([])
        self.surface_points = np.array([])

        self.subscription = self.create_subscription(
            PoseStamped,
            '/NDI/Pointer/measured_cp',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.server = pyigtl.OpenIGTLinkServer(port=18944)

        # Test Mode
        self.test_loadT = False
        self.test_saveT = True
        self.eps_mode = 0


    def listener_callback(self, msg):
        # self.ndi_pose = msg
        # R_m = self.quaternion_to_rotation_matrix([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        # t_m = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        # self.ndi_loc = np.dot(R_m, self.pointer) + t_m
        self.ndi_loc = np.array([msg.pose.position.x*1000, msg.pose.position.y*1000, msg.pose.position.z*1000])
        # self.get_logger().info(f"Received pose: {self.ndi_loc}")
        # self.read_ref_markers_ts()
        if self.if_surface :
            if self.surface_points.size == 0:
                self.surface_points = np.dot(self.R, self.ndi_loc) + self.t
                self.get_logger().info(f"Record the shape point: {self.ndi_loc}")
            else:
                self.surface_points = np.row_stack((self.surface_points, np.dot(self.R, self.ndi_loc) + self.t))
                self.get_logger().info(f"Record the shape point: {self.ndi_loc}")
    
    def quaternion_to_rotation_matrix(self, q):
        """Convert a quaternion into a rotation matrix."""
        q = np.array(q, dtype=np.float64)
        n = np.dot(q, q)
        if n < np.finfo(q.dtype).eps:
            return np.identity(3)
        q *= np.sqrt(2.0 / n)
        q = np.outer(q, q)
        return np.array([
            [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0]],
            [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0]],
            [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2]]
        ])
    
    def read_ref_markers_ct(self, file_path):
        if self.test_loadT:
            self.R = np.loadtxt("R.txt")
            self.t = np.loadtxt("t.txt")
            self.get_logger().info(f"Read the transformation matrix successfully!")
            return
        with open(file_path, 'r') as file:
            data = json.load(file)
        
        control_points = data['markups'][0]['controlPoints']
        
        positions = []
        
        for point in control_points:
            positions.append(point['position'])
        
        positions_array = np.array(positions)
        
        self.ref_points_ct = positions_array
        self.get_logger().info(f"Read the reference points of the CT successfully!")
        self.pyigtl_send_point(self.ref_points_ct[0], group_name='Reference Point', point_name = '1')

    
    def read_ref_markers_ts(self):
        if self.ref_points_ct.size == self.ref_points_ts.size:
            self.get_logger().info(f"Reference points of the CT and the TS have been matched!")
            return
        # Read the reference points of the target surface
        # ref_point_ts = self.ndi_pose
        if self.ref_points_ts.size == 0:
            self.ref_points_ts = self.ndi_loc
        else:
            self.ref_points_ts = np.row_stack((self.ref_points_ts, self.ndi_loc))
        # self.ref_points_ts.append([ref_point_ts.pose.position.x, ref_point_ts.pose.position.y, ref_point_ts.pose.position.z])
        self.get_logger().info(f"Read the reference point: {self.ndi_loc}!")
        if self.ref_points_ct.size != self.ref_points_ts.size:
            self.pyigtl_send_point(self.ref_points_ct[self.ref_points_ts.shape[0]], group_name='Reference Point', point_name = str(self.ref_points_ts.shape[0]))
    
    def delete_ref_markers_ts(self):
        if self.ref_points_ts.size == 0:
            return
        self.ref_points_ts = np.delete(self.ref_points_ts, -1, axis=0)
        self.get_logger().info(f"Delete the last reference point of the points list")
        self.pyigtl_send_point(self.ref_points_ct[self.ref_points_ts.shape[0]], group_name='Reference Point', point_name = str(self.ref_points_ts.shape[0]))
    
    def calculate_transform_matrix(self):
        # Calculate the transformation matrix
        if len(self.ref_points_ct) != len(self.ref_points_ts):
            self.get_logger().info(f"Reference points of the CT and the TS do not match!")
            self.get_logger().info(f"Reference points of the CT: {len(self.ref_points_ct)}")
            self.get_logger().info(f"Reference points of the TS: {len(self.ref_points_ts)}")
            return
        else:
            centroid_ts = np.mean(self.ref_points_ts, axis=0)
            centroid_ct = np.mean(self.ref_points_ct, axis=0)
            ref_points_ts_centered = self.ref_points_ts - centroid_ts
            ref_points_ct_centered = self.ref_points_ct - centroid_ct

            H = np.dot(ref_points_ts_centered.T, ref_points_ct_centered)
            U, S, Vt = svd(H)
            self.R = np.dot(Vt.T, U.T)
            if np.linalg.det(self.R) < 0:
                Vt[2,:] *= -1
                self.R = np.dot(Vt.T, U.T)
            self.t = centroid_ct - np.dot(self.R, centroid_ts)
            self.get_logger().info(f"Calculate the transformation matrix successfully!")
            if self.test_saveT:
                np.savetxt("R.txt", self.R)
                np.savetxt("t.txt", self.t)

    def read_tar_markers_ts(self):
        # Read the target points of the target surface
        tar_point_ts = self.ndi_loc
        if self.tar_points_ts.size == 0:
            self.tar_points_ts = np.array([tar_point_ts])
            self.tar_points_ct = np.array([np.dot(self.R, tar_point_ts) + self.t])
        else:
            self.tar_points_ts = np.row_stack((self.tar_points_ts, tar_point_ts))
            self.tar_points_ct = np.row_stack((self.tar_points_ct, np.dot(self.R, tar_point_ts) + self.t))
        # self.tar_points_ts.append([tar_point_ts.pose.position.x, tar_point_ts.pose.position.y, tar_point_ts.pose.position.z])
        self.get_logger().info(f"Read the target point of the TS: {tar_point_ts}!")
        self.get_logger().info(f"Calculate the target point of the CT: {np.dot(self.R, tar_point_ts) + self.t}!")
        tar_names = []
        for i in range(self.tar_points_ts.shape[0]):
            tar_names.append(str(i+1))
        self.pyigtl_send_point(self.tar_points_ct, group_name='Target Points', point_name = tar_names, color = [0, 255, 0, 255])
    
    def delete_tar_markers_ts(self):
        self.tar_points_ts = np.delete(self.tar_points_ts, -1, axis=0)
        self.tar_points_ct = np.delete(self.tar_points_ct, -1, axis=0)
        self.get_logger().info(f"Delete the last target point of the points list")
        tar_names = []
        for i in range(self.tar_points_ts.shape[0]):
            tar_names.append(str(i+1))
        self.pyigtl_send_point(self.tar_points_ct, group_name='Target Points', point_name = tar_names, color = [0, 255, 0, 255])


    def convert_lps_to_ras(self, lps_points):
        # Check if the input is a single point or multiple points
        # and convert using numpy operations to ensure output is also a numpy array
        if lps_points.ndim == 1:
            # Single point conversion
            ras_point = np.array([-lps_points[0], -lps_points[1], lps_points[2]])
        elif lps_points.ndim == 2:
            # Multiple points conversion
            ras_point = np.copy(lps_points)  # Create a copy to avoid modifying the original data
            ras_point[:, 0] = -lps_points[:, 0]  # Negate the first column (x)
            ras_point[:, 1] = -lps_points[:, 1]  # Negate the second column (y)
        else:
            raise ValueError("Invalid input shape: Expected 1D or 2D array.")
        
        return ras_point
    
    def ellipsoid_start(self):
        if self.eps_mode == 0:
            self.eps_start = self.tar_points_ct.shape[0]
            self.get_logger().info(f"Start record ellipsoid!")
        elif self.eps_mode == 1:
            self.if_surface = True
            self.get_logger().info(f"Start record ellipsoid trajactory!")
    
    def fit_ellipsoid(self, points):
        # Move points to the mean center
        center = np.mean(points, axis=0)
        centered_points = points - center
        
        # Fit the form Ax^2 + By^2 + Cz^2 + D = 1
        D = np.column_stack((centered_points[:, 0]**2, centered_points[:, 1]**2, centered_points[:, 2]**2))
        v = np.linalg.lstsq(D, np.ones(len(centered_points)), rcond=None)[0]
        
        # Compute the axes lengths
        a = 1 / np.sqrt(v[0])
        b = 1 / np.sqrt(v[1])
        c = 1 / np.sqrt(v[2])
    
        return a, b, c, center

    def create_ellipsoid_mesh(self, a, b, c, center, resolution=40):
        """Create an ellipsoid mesh using open3d."""
        mesh = o3d.geometry.TriangleMesh.create_sphere(radius=1, resolution=resolution)
        scale = np.array([a, b, c])
        mesh.vertices = o3d.utility.Vector3dVector(np.asarray(mesh.vertices) * scale)  # Apply scaling to vertices
        mesh.translate(center)
        return mesh 

    def convert_o3d_mesh_to_vtk(self, mesh):
        """Convert an Open3D mesh to a VTK mesh format."""
        # Extract vertices and faces from Open3D mesh
        vertices = np.asarray(mesh.vertices)
        faces = np.asarray(mesh.triangles)

        # Create a VTK Points object
        points = vtk.vtkPoints()
        for vertex in vertices:
            points.InsertNextPoint(*vertex)

        # Create a VTK PolyData object
        polydata = vtk.vtkPolyData()
        polydata.SetPoints(points)

        # Create VTK cells for faces
        cells = vtk.vtkCellArray()
        for face in faces:
            triangle = vtk.vtkTriangle()
            triangle.GetPointIds().SetId(0, face[0])
            triangle.GetPointIds().SetId(1, face[1])
            triangle.GetPointIds().SetId(2, face[2])
            cells.InsertNextCell(triangle)

        polydata.SetPolys(cells)
        return polydata

    def ellipsoid_end(self):
        if self.eps_start == -1:
            self.get_logger().info(f"Please start record ellipsoid first!")
            return
        self.get_logger().info(f"End record ellipsoid!")
        self.eps_count += 1
        ellipsoid_points_org = self.tar_points_ct[self.eps_start:]
        self.eps_start = -1
        ellipsoid_points = self.convert_lps_to_ras(ellipsoid_points_org)
        a, b, c, center = self.fit_ellipsoid(ellipsoid_points)
        ellipsoid_mesh = self.create_ellipsoid_mesh(a, b, c, center)
        elliposid_vtk = self.convert_o3d_mesh_to_vtk(ellipsoid_mesh)
        message = pyigtl.PolyDataMessage(elliposid_vtk, device_name='Tissue'+str(self.eps_count))
        self.server.send_message(message, wait=True)
    
    def surface_start(self):
        self.if_surface = True
        self.get_logger().info(f"Start record surface trajactory!")
    
    def surface_end(self):
        self.if_surface = False
        if self.surface_points.size == 0:
            self.get_logger().info(f"Please record the surface trajactory first!")
            return
        self.get_logger().info(f"End record surface trajactory!")
        self.eps_count += 1
        surface_points = self.convert_lps_to_ras(self.surface_points)
        self.surface_points = np.array([])
        # Deprecated: Use Open3D to create a mesh with ball pivoting
        # point_cloud = o3d.geometry.PointCloud()
        # point_cloud.points = o3d.utility.Vector3dVector(surface_points)
        # point_cloud.estimate_normals()
        # distances = point_cloud.compute_nearest_neighbor_distance()
        # avg_dist = np.mean(distances)
        # std_dev = np.std(distances)
        # radii = [avg_dist - 0.5 * std_dev, avg_dist, avg_dist + 0.5 * std_dev]
        # surface_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        #     point_cloud,
        #     o3d.utility.DoubleVector(radii)
        # )
        # # save the mesh
        # o3d.io.write_triangle_mesh("surface_mesh.ply", surface_mesh)
        a, b, c, center = self.fit_ellipsoid(surface_points)
        surface_mesh = self.create_ellipsoid_mesh(a, b, c, center)
        surface_vtk = self.convert_o3d_mesh_to_vtk(surface_mesh)
        message = pyigtl.PolyDataMessage(surface_vtk, device_name='Tissue'+str(self.eps_count))
        self.server.send_message(message, wait=True)

    def pyigtl_send_point(self, lps_point, group_name='Markers', point_name='1', color = [255, 0, 0, 255]):
        # First convert LPS point to RAS point
        ras_point = self.convert_lps_to_ras(lps_point)
        
        # Prepare and send the message
        output_message = pyigtl.PointMessage(
            device_name=group_name, positions=ras_point,
            names=point_name, rgba_colors=color)
        self.server.send_message(output_message, wait=True)

    def reset_demo(self):
        self.tar_points_ts = np.array([])
        self.tar_points_ct = np.array([])
        self.shape_points = np.array([])
        self.surface_points = np.array([])
        self.get_logger().info(f"Reset the demo successfully!")