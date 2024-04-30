import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np

class PivotCalibration(Node):
    def __init__(self):
        super().__init__('pivot_calibration')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/NDI/Pointer/measured_cp',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Initialize a list to store transformation matrices
        self.transformation_matrices = []

    def listener_callback(self, msg):
        # Extract position and orientation from PoseStamped message
        position = msg.pose.position
        orientation = msg.pose.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        R = self.quaternion_to_rotation_matrix(q)
        t = np.array([position.x, position.y, position.z, 1])

        # Form the transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t[:3]

        # Store the transformation matrix
        self.transformation_matrices.append(T)

        # Perform calibration when enough data has been collected
        if len(self.transformation_matrices) > 3000:
            self.calculate_pivot_calibration()

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

    def calculate_pivot_calibration(self):
        # Convert list to a numpy array for processing
        tracking_matrices = np.array(self.transformation_matrices)

        # Algebraic One Step method for pivot calibration
        pointer_offset, pivot_location, residual_error = self.pivot_calibration_aos(tracking_matrices)

        # Save the results to files
        np.savetxt("pointer_offset.txt", pointer_offset, fmt='%f')
        np.savetxt("pivot_location.txt", pivot_location, fmt='%f')

        # Clear the list after processing
        self.transformation_matrices = []

        print(f"Pivot calibration completed with residual error: {residual_error}")

    def pivot_calibration_aos(self, tracking_matrices):
        # Constructing A and b matrices for the algebraic approach
        A = np.zeros((6 * len(tracking_matrices), 6))
        b = np.zeros(6 * len(tracking_matrices))
        for i, T in enumerate(tracking_matrices):
            R = T[:3, :3]
            t = T[:3, 3]
            A_block = np.eye(3, 6)
            A_block[:, 3:] = -R
            A[3*i:3*i+3, :] = A_block
            b[3*i:3*i+3] = t

        # SVD decomposition and solving
        u, s, vh = np.linalg.svd(A, full_matrices=False)
        c = np.dot(u.T, b)
        w = np.dot(np.diag(1 / s), c)
        x = np.dot(vh.T, w)

        pointer_offset = x[:3]
        pivot_location = x[3:6]
        residual_error = np.linalg.norm(np.dot(A, x) - b) / np.sqrt(len(b))  # RMS error

        return pointer_offset, pivot_location, residual_error

def main(args=None):
    rclpy.init(args=args)
    pivot_calibration_node = PivotCalibration()
    rclpy.spin(pivot_calibration_node)
    pivot_calibration_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
