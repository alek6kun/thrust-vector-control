addpath(fullfile('..', 'src'));
addpath(fullfile('..', 'MpcControl/'));
close all
clear
clc

%% Initialisations

Ts = 1/20;
rocket = Rocket(Ts);

[xs,us] = rocket.trim(); 
sys = rocket.linearize(xs,us); 

[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys,xs,us);
Tf = 15;
H = 7.0; %[s] Horizon length
rocket.anim_rate = 10; %increase this to make animation faster

mpc_x = MpcControl_x(sys_x,Ts,H);
mpc_y = MpcControl_y(sys_y,Ts,H);
mpc_z = MpcControl_z(sys_z,Ts,H);
mpc_roll = MpcControl_roll(sys_roll,Ts,H);

%Merge four sub-system controllers into one full-system controller
mpc = rocket.merge_lin_controllers(xs,us,mpc_x,mpc_y,mpc_z,mpc_roll);
% Manipulate mass for simulation
rocket.mass = 2.13;

x0 = [zeros(1, 9), 1 0 3]';
ref = [1.2, 0, 3, 0]';

%% Simulation and plot with estimator
[T_est, X_est, U_est, Ref_est, Z_hat] = ...
    rocket.simulate_est_z(x0, Tf, @mpc.get_u, ref, mpc_z, sys_z);
 
ph_no_est = rocket.plotvis(T_no_est,X_no_est,U_no_est,Ref_no_est);
ph_no_est.fig.Name = 'Merged lin. MPC in nonlinear simulation without estimator';
saveas(gcf,'5.1 without estimator.png')

%% Simulation and plot without estimator
[T_no_est, X_no_est, U_no_est, Ref_no_est] = ...
    rocket.simulate(x0, Tf, @mpc.get_u, ref);
ph_est = rocket.plotvis(T_est,X_est,U_est,Ref_est);
ph_est.fig.Name = 'Merged lin. MPC in nonlinear simulation with estimator';
saveas(gcf,'5.1 with estimator.png')
