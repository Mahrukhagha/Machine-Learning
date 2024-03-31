% This is the main script for running the ekf slam algorithm.
clc; clear;
close all;

% add path to utils
addpath('./utils');

% load dataset
data_path = '../data';
dataset_name = 'square_shape_without_noise';
load([data_path,'/',dataset_name,'.mat']);

%% Define noise params
% 3x3 process noise
sigma.x = 0.1; % [m]
sigma.y = 0.1; % [m]
sigma.theta = deg2rad(1); % [rad]
params.R = diag([sigma.x, sigma.y, sigma.theta].^2);

% 2x2 observation noise
sigma.r = 0.1; % [m]
sigma.phi = deg2rad(1); % [rad]
params.Q = diag([sigma.r, sigma.phi].^2);

%% Initialize state variable
state.mu = data{1}.gt.pose;
state.cov = zeros(3,3);
state.num_lm = 0;
state.s_lm = [];
state.ind_lm = {};

%% Data association
use_known_da = true;  % True when using landmark ids for landmark association
correct_every = 1;    % Correction step applied to every nth step

%% visualization
plot_covariance_matrix = false;
fh = figure;
if(plot_covariance_matrix)
    fh_cov = figure;
end

%% Main Loop
num_steps = length(data);
del_t = 0.1;
for i = 2 :  num_steps
    
    % read control
    u = [data{i-1}.control.v, data{i-1}.control.omega]';
    
    % ekf prediction step
    state = ekf_predict(state, u, del_t, params.R);
        
 
    % read measurement
    z = data{i}.measurements.z;
    if(~use_known_da)
        z = z(1:2,:);
    end
    
    % ekf correction step
    if(mod(i,correct_every)==0)
       state = ekf_correct(state, z, params.Q);
    end
    
    % visualize robot pose, landmarks and their covariances
    if mod(i,1)==0
         show_visualization(fh, state, z, M, data{i}, false);
%         
         if(plot_covariance_matrix)
             plot_state_covariance(fh_cov, state.cov, size(M.X,1));
         end
     end
  
end