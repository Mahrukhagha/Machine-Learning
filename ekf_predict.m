% This function performs the prediction step of the EKF SLAM algorithm
% Input: 
%       state: struct containing state variables
%           mu  =  state vector containing 2D robot pose and 2D lanmark locations
%                  [x, y, theta, l1x, l1y, ..... lnx, lny]' [(3+2N),1]
%           cov =  covariance matrix [(3+2N), (3+2N)]
%           s_lm = signature/id of the landmark
%           num_lm = number of landmarks currently in the state vector
%           ind_lm =  num_lm cell array containing indices of landmark i
%                   ind_lm{i}(1) = index for the x coordinate of landmark i   
%                   ind_lm{i}(2) = index for the y coordinate of landmark i
%       u: control command
%           u(1) = linear  velocity
%           u(2) = angular velocity
%       del_t: duration for which the control is applied
%       R: process noise [3,3] 
% Output:
%       state: updated state structure after prediction step
function state = ekf_predict(state, u, del_t, R)

% TODO: Define F (matrix taking care that only robot state is updated)
%state.num_lm = 1;
F = [ eye(3,3) , zeros(3,2*state.num_lm) ] ;


% TODO: update mean and covariance 
%% intialize
%First state is known from state.mu
%Arrange pose  and landmarks in order [x, y, theta, l1x, l1y, ..... lnx, lny]' [(3+2N),1]
%state_x = state
%state.mu(4:63,1) = zeros(2*M.num_landmarks,1)

%size of state vector
size_s_v = size(state.mu,1);
%for i = 1 : state.num_lm
theta =(state.mu(3,1)); 

% for angular velocity approaches to zero   
if (u(2) < 10^-3)
    
state.mu = state.mu + F' * [state.mu(1)+u(1)*cos((theta))*del_t;...
                              state.mu(2)+u(1)*sin((theta))*del_t;...
                               (state.mu(3))];
state.mu(3) = normalize_angle(state.mu(3));
G = eye(size_s_v) + F' *  [1 ,0, -u(1)*sin((theta))*del_t;...
                            0, 1, u(1)*cos((theta))*del_t;...
                             0 ,0 ,1] * F;
else


%Updated in intialize_new _landmark state.mu(4:2*state.num_lm+3)  = [ state.ind_lm{i}(1) ; state.ind_lm{i}(2)];
state.mu = state.mu + F' * [-u(1)/u(2)*sin((theta)) + u(1)/u(2)*sin((theta) + u(2)*del_t),...
                             u(1)/u(2)*cos((theta)) - u(1)/u(2)*cos((theta) + u(2)*del_t),...
                             u(2)*del_t]';
%Compute the 3x3 Jacobian Gx of the motion model                        
G = eye(size_s_v) + F' * [zeros(3,2) [-u(1)/u(2)*cos((theta)) + u(1)/u(2)*cos((theta) + u(2)*del_t);...
                             -u(1)/u(2)*sin((theta)) + u(1)/u(2)*sin((theta) + u(2)*del_t);...
                              0]] * F;
end
%if problem check R size                   
state.cov = G * state.cov * G' + F' * R * F;
end
