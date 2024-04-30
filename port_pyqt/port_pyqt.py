import sys
import rclpy
from . ui_port import *
from PyQt5.QtWidgets import QApplication, QMainWindow
from . port_node import objectClient
import threading  

# pyuic5 -o ui_port.py port.ui

class MyApp(QMainWindow):
    def __init__(self, node):
        QMainWindow.__init__(self)
        self.node = node

        # UI
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        # Bind the button click event
        self.ui.loadPoseButton.clicked.connect(self.read_ref)
        self.ui.recordRegPointButton.clicked.connect(self.node.read_ref_markers_ts)
        self.ui.deleteRegPointButton.clicked.connect(self.node.delete_ref_markers_ts)
        self.ui.calcTButton.clicked.connect(self.node.calculate_transform_matrix)
        self.ui.recordPointButton.clicked.connect(self.node.read_tar_markers_ts)
        self.ui.deletePointButton.clicked.connect(self.node.delete_tar_markers_ts)
        self.ui.startShapeButton.clicked.connect(self.node.ellipsoid_start)
        self.ui.stopShapeButton.clicked.connect(self.node.ellipsoid_end)
        self.ui.pushButton.clicked.connect(self.node.surface_start)
        self.ui.pushButton_2.clicked.connect(self.node.surface_end)
        self.ui.pushButton_3.clicked.connect(self.node.reset_demo)

        t_ = threading.Thread(target=self.thread_ros, args=())
        t_.setDaemon(True)
        t_.start()

    def thread_ros(self):
        rclpy.spin(self.node)
        self.node.destroy_node()
        rclpy.shutdown()
    
    def read_ref(self):
        Filepath = QtWidgets.QFileDialog.getOpenFileName(self,  "Select Reference YAML file","./", "JSON (*.json)")
        self.node.read_ref_markers_ct(Filepath[0]) 
        

def main(args=None):
    rclpy.init(args=args)                                 # ROS2 Python init
    node = objectClient("port_application")       # Create and initialize the ROS2 node object

    # UI init
    app = QApplication(sys.argv)
    myapp = MyApp(node)
    myapp.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
