addpath(fullfile('..', 'src'));
close all
clear all
clc

%% Intialisations
Ts = 1/20;
rocket = Rocket(Ts);
H = 2; % Horizon length in seconds
nmpc = NmpcControl(rocket, H);
x0 = zeros(12,1);

%% Open loop simulation

% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
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
rocket.anim_rate = 10;
ph2 = rocket.plotvis(T, X, U, Ref);
ph2.fig.Name = 'NMPC in simulation';

%% Closed loop simulation with 50 degrees max roll

roll_max = deg2rad(50);
ref = @(t_, x_) ref_TVC(t_, roll_max);

% Simulate
Tf = 30;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
rocket.anim_rate = 10;
ph3 = rocket.plotvis(T, X, U, Ref);
ph3.fig.Name = 'NMPC in simulation, roll_max 50 degrees';

%% Comparison with linear controller with max roll of 50 degrees

%Compute steady-state for which 0 = f(xs,us)
[xs,us] = rocket.trim();

%Linearize the nonlinear model about trim point
sys = rocket.linearize(xs,us); 

[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys,xs,us);

H = 2; %[s] Horizon length

%Controller for system x:
mpc_x = MpcControl_x(sys_x,Ts,H);
mpc_y = MpcControl_y(sys_y,Ts,H);
mpc_z = MpcControl_z(sys_z,Ts,H);
mpc_roll = MpcControl_roll(sys_roll,Ts,H);

%This comes from the data 
%Merge fous sub-system controllers into one full-system controller
mpc = rocket.merge_lin_controllers(xs,us,mpc_x,mpc_y,mpc_z,mpc_roll);

%Simulate
Tf = 30;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);

%Visualize
rocket.anim_rate = 10; %increase this to make animation faster
ph4 = rocket.plotvis(T,X,U,Ref);
ph4.fig.Name = 'Merged lin. MPC in nonlinear simulation';


