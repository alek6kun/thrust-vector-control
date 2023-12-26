addpath(fullfile('..', 'src'));
addpath(fullfile('..', '..', 'casadi-3.6.4-windows64-matlab2018b'));
%close all
%clear all
%clc

%% Intialisations
Ts = 1/20;
rocket = Rocket(Ts);
H = 3; % Horizon length in seconds
nmpc = NmpcControl(rocket, H);

%% Open loop simulation

% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
x0 = zeros(12,1);
ref4 = [2; 2; 10; deg2rad(40)];
[u, T_opt, X_opt, U_opt] = nmpc.get_u(x0, ref4);
U_opt(:,end+1) = nan;
ph1 = rocket.plotvis(T_opt, X_opt, U_opt, ref4);
ph1.fig.Name = 'Optimal open loop trajectory';

%% Closed loop simulation with default max roll

% Setup MPC reference function with default maximum roll = 15 deg
ref = @(t_, x_) ref_TVC(t_);

% Simulate
Tf = 30;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
rocket.anim_rate = 2.0;
ph2 = rocket.plotvis(T, X, U, Ref);
ph2.fig.Name = 'NMPC in simulation';

%% Closed loop simulation with default max roll

roll_max = deg2rad(50);
ref = @(t_, x_) ref_TVC(t_, roll_max);

% Simulate
Tf = 30;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
rocket.anim_rate = 2.0;
ph3 = rocket.plotvis(T, X, U, Ref);
ph3.fig.Name = 'NMPC in simulation, roll_max 50 degrees';

%% Comparison with linear controller with max rool of 50 degrees

% To do
