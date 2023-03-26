<!DOCTYPE html>
<html>
<head>
</head>
<body>
	<h1>Introduction</h1>
	<p>This is a MATLAB code designed to fuse data from lidar, radar, and stereoscopic camera measurements to estimate the distance of an object from a sensor. The code implements a Kalman filter to process the measurements and estimate the object distance.</p>
  <h2>Getting Started</h2>
<p>To use this code, first download or clone the repository to your local machine. You will need MATLAB installed to run the code.</p>

<h2>Usage</h2>
<p>The main script is kalman_filter.m, which contains the implementation of the Kalman filter. To run the script, simply open it in MATLAB and click the "Run" button. You can modify the parameters of the Kalman filter, such as the measurement noise covariance matrix, in the script to optimize its performance.</p>

<h2>Data</h2>
<p>The code uses data from four files: exp2jose.mat, AutoResExp2.mat, exp2lazim.mat, and exp2_gt_josejuan_bis.mat. These files contain the measurements from the lidar, radar, and stereoscopic camera sensors, as well as the ground truth measurements of the object distance. You can replace these files with your own data if desired.</p>

<h2>Resampling</h2>
<p>The code includes a resampling step to resample the data to a common time base. The desired length for the resampled matrix can be modified in the script.</p>

<h2>Interpolation</h2>
<p>The code also includes an interpolation step to interpolate the data to a common time base. The time step for the interpolation can be modified in the script.This resampling is performed in different ways depending on the piece of code chosen, more specifically it is done linearly (combinewithkalman5puerlylinearexp2.m), non-linearly (combinewithkalman5nonlinearexp2.m), and with no extrapolation, raw data readings (combinewithkalman5puerlylinearexp2.m).</p>

<h2>Kalman Filter</h2>
<p>The Kalman filter implementation in the script includes prediction and update steps based on measurements from the lidar, radar, and stereoscopic camera sensors. The state estimate is updated using the Kalman gain, which is calculated based on the measurement noise covariance matrix and the state estimate covariance matrix.</p>

<h2>Value Estimation</h2>
<p>The "kalman_filter_nonuniform" function at the end of the code implements the Kalman filter. It takes lidar, radar, and camera measurements, as well as their respective time stamps, and returns the estimated range, velocity, and orientation. The depth and lateral distance are also provided as x(1) and x(2).</p>
 
<h2>Error Calculation</h2>
<p>The code calculates the mean square error and cross-correlation coefficient between the estimated object distance and the ground truth measurements, as well as the measurements from the lidar, radar, and stereoscopic camera sensors.</p>

<h2>Plotting</h2>
<p>The code includes a plot of the estimated object distance over time, as well as the ground truth measurements and the measurements from the lidar, radar, and stereoscopic camera sensors.</p>
	
<h2>Other sensor fusion kalman filters</h2>
<p>This repository includes code for sensor fusion with lidar and camera measurements, as well as radar and camera measurements. Additionally, it includes individual Kalman filter implementations for each of the sensors.</p>
<p>The lidar and camera fusion code is located in the file "lidarcamerapurelylinearexp2.m", and the radar and camera fusion code is located in the file "radarcamerapurelylinearexp2.m". The individual Kalman filter implementations for lidar, camera, and radar are located in the files "lidarpurelylinearexp2.m", "camerapurelylinearexp2.m", and "radarpurelylinearexp2.m", respectively.</p>
<p>These implementations use similar steps to the main Kalman filter implementation for sensor fusion, including resampling, interpolation, prediction, and update steps. The individual Kalman filters are useful for applications where only one sensor is available or for comparison with the fused sensor data.</p>

<h2>License</h2>
<p>This code is provided under the MIT License.</p>

<h2>Contributing</h2>
<p>Contributions to this code are welcome. To contribute, please fork the repository, make your changes, and submit a pull request.</p>

<h2>Contact</h2>
<p>If you have any questions or issues with this code, please contact the author at zceejde@ucl.ac.uk.</p>
</body>
</html>
