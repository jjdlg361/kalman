close all
clc
clear all

%% Exp 2 Walking Simple stop interval every 2m data combined
load('exp2jose.mat');
load('AutoResExp2.mat');
load('exp2lazim.mat')
load('exp2_gt_josejuan_bis.mat')

t_off = 21.5;
lt_off = 23.7;
d_off = 51; % constant offset cam and radar
ld_off = 78; % constant offset lidar to radar

lidar_time=times+lt_off;
radar_time = (time_frame)+t_off;
distance = (distance);
ranges = (ranges*1000) + ld_off; 

timet=time+0.5;
distancet=exp2_gt;

%% Kalman implementation

%% Measurements
t_lidar = times+lt_off;
for p=1:length(t_lidar)
    x_lidar(p) = distance(p) .* (cos(angles(p))); % meters
    y_lidar(p) = distance(p) .* (sin(angles(p))); % meters
end

%% Calculate radar quantities
range_rate = zeros(length(radar_time)-1,1);
gearing_rate = zeros(length(radar_time)-1,1);
for i = 1:length(radar_time)-1
    delta_range = distance(i+1) - distance(i);
    delta_azimuth = azimuth(i+1) - azimuth(i); % assuming uniform time steps of 1/240 seconds
    delta_time=time_frame(i+1)-time_frame(i);
    range_rate(i) = delta_range/delta_time;
    gearing_rate(i) = (velocity(i+1) - velocity(i))/(range_rate(i)*delta_time);
end

lidar_meas = transpose([x_lidar;y_lidar]);
lidar_meas_2=[ranges'/1000,angles'];
lidar_meas_noise = 0.15;
t_radar = radar_time(:,2:end);
radar_meas = [distance(2:end,:),transpose(azimuth(2:end)),range_rate,transpose(velocity(2:end))];
radar_meas_noise = 0.1;
t_camera = tempos_def;
camera_meas = transpose([x_coord_def(1:77)/1000;z_coord_def/1000;alpha_coord_def(1:77)]);%(dist_point_coord_def-d_off)/1000;
camera_meas_noise = 0.15;
% Set initial state and covariance estimates
pos_var_init = 0.328;
vel_var_init = 0.01;
ori_var_init = 0.0;
%% Resampling

%% Lidar
% Define the desired length for the resampled matrix
desired_length = length(radar_meas)-1; % Replace with your desired length

% Determine the resampling factor
resampling_factor_lidar = desired_length / length(lidar_meas);

% Resample the input matrix to the desired length
[lidar_resampled,rtl] = resample(lidar_meas,t_lidar,  1);

% %% Stereo
% % Determine the resampling factor
% resampling_factor_stereo =  desired_length/ length(t_camera);
% 
% % Resample the input matrix to the desired length
% [camera_resampled,rts] = resample(camera_meas, t_camera, resampling_factor_lidar);
% 
% % %% Radar 
% % [radar_resampled,rtr] = resample(radar_meas(:,2:end), radar_time(:,2:end), 20);
% % radar_meas=radar_resampled;
% % t_radar=rtr;
%% Lidar discrete interpolation
dt = 0.2;
% Signals
t4 = lidar_time+2.8;
x4 = lidar_meas_2;
% Interpolation
tbis_base = t4(1):dt: t4(end);
x4_interp_base = interp1(t4, x4, tbis_base, 'linear', 'extrap');

%% Stereo discrete interpolation
t6 = t_camera;
x6 = camera_meas(:,1);
x6_1=camera_meas(:,2);
x6_2=camera_meas(:,3);
% Make datareadings unique
[t6_unique, idx_unique] = unique(t6);
x6_unique = x6(idx_unique);
x6_1_unique = x6_1(idx_unique);
x6_2_unique = x6_2(idx_unique);
tbis_stereo_baseline = t6_unique(1):dt:t6_unique(end);
% Interpolate signals to a common time base
x6_interp = interp1(t6_unique, x6_unique, tbis_stereo_baseline, 'linear', 'extrap')';
x6_1_interp = interp1(t6_unique, x6_1_unique, tbis_stereo_baseline, 'linear', 'extrap')';
x6_2_interp = interp1(t6_unique, x6_2_unique, tbis_stereo_baseline, 'linear', 'extrap')';
%% Run Kalman filter
[range_estimate, velocity_estimate, orientation_estimate] = kalman_filter_nonuniform(radar_meas,t_radar(2:end), pos_var_init, vel_var_init, ori_var_init, lidar_meas_noise, radar_meas_noise, camera_meas_noise);
%% Error calculation
%% Lidar
t3 = t_radar(2:end);
x3 = range_estimate;

% Interpolate signals to a common time base
tbis = max(t3(1), t4(1)):dt:min(t3(end), t4(end));
x3_interp = interp1(t3, x3, tbis, 'linear', 'extrap');
x4_interp = interp1(t4, x4, tbis, 'linear', 'extrap');

% Calculate mean square error
x4_mm=x4_interp(:,1);
mse_lidar = mean((x3_interp - x4_mm').^2);

% Compute cross-correlation coefficient
rxy_lidar = corrcoef(x3_interp, x4_interp(:,1));
rxy_lidar = rxy_lidar(1, 2);
%% Radar
errvsrada = immse(range_estimate,distance(3:end,:));

% Compute cross-correlation coefficient

% Compute cross-correlation coefficient
rxy_radar = corrcoef(range_estimate,distance(3:end,:));
rxy_radar = rxy_radar(1, 2);
%% Ground
% Sample signals
t1 = t_radar(2:end);
x1 = range_estimate;
t2 = timet+0.1;
x2 = distancet;

% Interpolate signals to a common time base

t = max(t1(1), t2(1)):dt:min(t1(end), t2(end));
x1_interp = interp1(t1, x1, t, 'linear', 'extrap');
x2_interp = interp1(t2, x2, t, 'linear', 'extrap');

% Calculate mean square error
mse_ground = mean((x1_interp - x2_interp).^2);

% Compute cross-correlation coefficient
rxy = corrcoef(x1_interp, x2_interp);
rxy = rxy(1, 2);




%% Stereo

t6 = t_camera;
x6 = sqrt((x_coord_def(1:77)/1000).^2+(z_coord_def/1000).^2);
t5 = t_radar(2:end);
x5 = range_estimate;

% Make datareadings unique
[t5_unique, idx_unique] = unique(t5);
x5_unique = x5(idx_unique);
[t6_unique, idx_unique] = unique(t6);
x6_unique = x6(idx_unique);

% Interpolate signals to a common time base
tbis_stereo = max(t5_unique(1), t6_unique(1)):dt:min(t5_unique(end), t6_unique(end));
x5_interp = interp1(t5_unique, x5_unique, tbis_stereo, 'linear', 'extrap');
x6_interp = interp1(t6_unique, x6_unique, tbis_stereo, 'linear', 'extrap');

% Calculate mean square error
mse_stereo = mean((x5_interp - x6_interp).^2);
% Compute cross-correlation coefficient
rxy_Scamera = corrcoef(x5_interp, x6_interp);
rxy_Scamera = rxy_Scamera(1, 2);
%% Display
% Display results
disp(['Mean square error to ground truth: ' num2str(mse_ground)]);
disp(['Cross-correlation coefficient to ground truth: ' num2str(rxy)]);
disp(['Mean square error to radar reading: ' num2str(errvsrada)]);
disp(['Cross-correlation coefficient to radar reading: ' num2str(rxy_radar)]);
disp(['Mean square error to lidar reading: ' num2str(mse_lidar)]);
disp(['Cross-correlation coefficient to lidar reading: ' num2str(rxy_lidar)]);
disp(['Mean square error to stereoscopic camera reading: ' num2str(mse_stereo)]);
disp(['Cross-correlation coefficient to camera reading: ' num2str(rxy_Scamera)]);
%% Plotting
figure;
hold on;
%Plot estimate
plot(t_radar(2:end), range_estimate, '-o', 'LineWidth', 2);
hold on
%Plot ground truth
plot(timet+0.1, distancet,'-o');
hold on
%Plot stereo camera
plot(rts,sqrt(camera_resampled(:,1).^2+camera_resampled(:,2).^2));hold on
%Plot radar
plot(radar_time(:,2:end),distance(2:end,:));hold on
%PLot lidar
%plot(lidar_time,ranges/1000);hold on
%plot(rtl,sqrt(lidar_resampled(:,1).^2+lidar_resampled(:,2).^2));
plot(tbis, x4_interp)
xlabel('Time (s)');
ylabel('Distance (m)');
title('Object distance estimate');
legend('Estimation', 'Ground Truth', 'Stereo Camera','Radar','Lidar');

%% Function

function [range_estimate, velocity_estimate, orientation_estimate] = kalman_filter_nonuniform(radar_meas, radar_t, pos_var_init, vel_var_init, ori_var_init, lidar_meas_noise, radar_meas_noise, camera_meas_noise)
% Kalman filter for fusing measurements from lidar, radar, and stere;  % Average walking speed in m/s
dt = 0.2;
v_walk = 1.8;  % Average walking speed in m/s
F = [1 dt 0 0 0 0 0; 0 v_walk 0 0 0 0 0; 0 0 1 dt 0 0 0; 0 0 0 v_walk 0 0 0; 0 0 0 0 1 dt 0; 0 0 0 0 0 v_walk 0; 0 0 0 0 0 0 1];
q_pos = 0.16
%0.3^2;%0.1 good compromise the bigger the mse error the better is bigger q_pos
q_vel = 0.1^2;

Q = diag([q_pos, q_vel, q_pos, q_vel, 180, 180, 180]);
Q=Q;%Increse process noise, allow more change between meausurements

% Initialize state estimate vector x and covariance matrix P
x = [radar_meas(1, 1), 0, radar_meas(1, 2), 0, 0, 0, 0]';
P = diag([pos_var_init, vel_var_init, pos_var_init, vel_var_init, ori_var_init, ori_var_init, ori_var_init]);

% Define measurement noise covariance matrix R
% R = diag([lidar_meas_noise^2;
%          radar_meas_noise^2;
%          camera_meas_noise^2;
%          lidar_meas_noise^2;
%          radar_meas_noise^2;
%          camera_meas_noise^2; ...
%          lidar_meas_noise^2;
%          radar_meas_noise^2;
%          camera_meas_noise^2]);
R = diag([0.1^2;%Lidar
         0.1^2;%Lidar
         0.9^2;%Radar
         0.9^2;%Radar 0.1 prev
         0.1^2;%Radar
         0.1^2; ...%Placeholder
         0.1^2;%Camera
         0.1^2;%Camera 0.2 prev
         0.1^2]);%Camera
% Initialize estimates for position, velocity, and orientation
num_time_steps = length(radar_t);
range_estimate = zeros(num_time_steps, 1);
velocity_estimate = zeros(num_time_steps, 2);
orientation_estimate = zeros(num_time_steps, 1);

% Loop over time steps
for t = 2:num_time_steps
    
    % Perform prediction step
    x_pred = F * x;
    P_pred = F * P * F' + Q;
    P_pred=1*P_pred;%Increase kalmans filter gain
    
    % Update state estimate based on radar measurement
    if t <= size(radar_meas, 1) && radar_t(t) > radar_t(t-1)
        
        %H_radar = [range 0 0 0 0 0 0;                   0 range 0 0 0 0 0;                   0 0 range_rate 0 0 0 gearing_rate;                   0 0 0 range_rate 0 range 0];
        z_radar = [radar_meas(t, 1); radar_meas(t, 2); radar_meas(t, 3); 0];
        H_radar = [1 0 0 0 0 0 0;            0 2 0 0 0 0 0;            0 0 1 0 0 0 0;            0 0 0 1 0 0 0];
        R_radar = R(3:6, 3:6);
        K_radar = P_pred * H_radar' / (H_radar * P_pred * H_radar' + R_radar);

        x = x_pred + K_radar * (z_radar - H_radar * x_pred);
        P = (eye(7) - K_radar * H_radar) * P_pred;
    end

    % Extract position, velocity, and orientation estimates
    range_estimate(t,:) = [sqrt(x(1)^2 + x(2)^2)]; % Calculate range from position estimate
    velocity_estimate(t,:) = x(3:4)';
    orientation_estimate(t,:) = x(7)';
end
end
