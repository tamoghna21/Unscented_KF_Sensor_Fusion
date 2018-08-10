# Unsceneted Kalman Filter for bicycle tracking on the road
The objective of this project is to detect a bicyle travelling around a car. The car is fitted with Lidar and radar sensors. Noisy lidar and radar data measurements are avilable. An unsceneted Kalman Filter has been implemented for this purpose.


[//]: # (Image References)

[video1]: ./UKF_video.mov "VideoUKF"
[image1]: ./ukf_sim_ss.png "ukf1"

* An unscented Kalman filter using the constant turn rate and velocity magnitude (CTRV) motion model is implemented.

* Lidar measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles. The image and the video below shows what the simulator looks like when a c++ script is using its Kalman filter to track the object. A simulator provides the script the measured data (lidar and/or radar), and the script feeds back the measured estimation marker, and RMSE values from its Kalman filter.


![alt text][image1]


* [Here's a video of the performance of MPC algorithm in the simulator][video1]









#### Note: This Project is part of the Udacity Self Driving Car nanodegree program. Instructions related to environment setup and the simulator can be found [here](https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project)
