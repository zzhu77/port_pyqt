# Enabling High Precision Post-Operative Radiation Therapy (PORT) Delivery for Skull Based Cancers

<!-- [![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE) -->

## Description

A program that maps the location of specimens to the Post-Operative CT images with Polaris.

## Depencencies

 - ROS2 Humble
 - [jhu-saw/sawNDITracker](https://github.com/jhu-saw/sawNDITracker)
 - Python
 - 3D Slicer

## Installation

Assuming you have some basic knowledge about ROS2 and have build sawNDITracker as mentioned above.
```
pip install -r requirements.txt
colcon build
source install/setup.sh
```

## Usage

1. Connect Polaris to the computer with USB port.
2. Run sawNDITracker `ros2 run ndi_tracker ndi_tracker -j cis2.json`
3. Check if the Pointer is detected.
4. Open 3D Slicer
5. Start a OpenIGTLink Clinet with Port 18944 in OpenIGTLinkIF extension in 3D Slicer.
6. Load the Dicom file of Pre-Operative CT scan in 3D Slicer.
7. Run the port program `ros2 run port_pyqt port_ui` 

### Calculate Transformation
1. Click on 'Load Pose' and choose the json file that records the location of Reference markers in Pre-op CT image.
2. Now, touch the Reference marker that labeled in the 3D slicer, and click 'Record Ref'. The order of recording Reference marker DOES MATTER. You may delete the last record with 'Delete Ref'.
3. After recording all the ref markers, click 'Calc T', which will calculate the transmation from phantom to pre-operative ct image.

### Map Points
1. Move the Pointer to the point you want to record.
2. Click 'Record Tar'
3. The point will be mapped to the pre-op ct scan with a number indicating the order.
4. You can click on 'Delete Tar' to delete the last point.

### Map Volume with Points
1. Click 'Start Shape'
2. Move the Pointer to a point on the tissue you want to record.
3. Click 'Record Tar'
4. The point will be mapped to the pre-op ct scan with a number indicating the order.
5. Repeat step 2 to 4. Record at least 10 different points on the tissue. You can click on 'Delete Tar' to delete the last point if recorded wrongly.
6. Click 'Stop Shape'
7. The estimated area of the tissue will be mapped to the pre-op ct scan with a number indicating the order. (eg. 'Tissue 1')

### Map Volume with Trajectory
1. Move the pointer to the surface of the object first.
2. Click 'Start Surface'.
3. Move the pointer on the surface of the object.
4. Click 'Stop Surface'.
5. The estimated area of the tissue will be mapped to the pre-op ct scan with a number indicating the order. (eg. 'Tissue 2')

### Register the Post-Op scan to Pre-op scan
1. Load the Dicom file of Post-Operative CT scan in 3D Slicer.
2. Choose Registration - Genreal Registration Module.
3. Use Pre-op image as Fixed Image Volume, Post-op image as Moving Image Volume, choose create new volume for Output Image Volume.
4. Choose Rigid(6DOF) in Registration Phases.
5. Apply. Now you will have the registrated Post-Op scan and can see the mapped points and volumes on the Post-op scan.

## Contact

- Project Link: [CIS2](https://ciis.lcsr.jhu.edu/doku.php?id=courses%3A456%3A2024%3Aprojects%3A456-2024-17%3Aproject-17)