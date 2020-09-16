# Agriculture Mobile Robot Simulation

Authors: Diego Rojas, Hamid Jafarbiglu, Starvos Vougioukas

This project is a simulation of a mobile robot navigating and mapping a field of trees to estimate the location and diameters of the tree trunks. The goal of this simulation is to develop and test robot software that can be deployed on a physical mobile robot. To improve the accuracy of the robot software, noisy lidar, GPS, and compass sensors plugins were used to simulate noise found in actual hardware. As a result, an ektended kalman filter was used to estimate the pose of the robot when navigating and recording laser scan data. Image processing techniques were used to filter the occupancy grid and determine the location and diameters of the trees. This information is outputted and organized in a text file.  

# Demo of Robot Navigation and Mapping 

<p align="center">
<img src="https://github.com/Drojas251/Agriculture-Mobile-Robot-Simulation-/blob/master/media/ag_robot.gif" width="400">

- Youtube Video can be found here: 
  [`Simulation video`](https://youtu.be/Uv0dmANF4rU)
  
# Images 
The image on the left is the ground truth field of trees while the image on the right is the generated occupancy grid 

<p align="center">
<img src="https://github.com/Drojas251/Agriculture-Mobile-Robot-Simulation-/blob/master/media/ground%20truth.JPG" width="400">
<img src="https://github.com/Drojas251/Agriculture-Mobile-Robot-Simulation-/blob/master/media/scanned_image.JPG" width="400">
