addpath(fullfile('..', 'src'));

%close all
%clear all
%clc

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20;
rocket = Rocket(Ts);

[xs,us] = rocket.trim(); %Compute steady-state for which 0 = f(xs,us)
sys = rocket.linearize(xs,us); %Linearize the nonlinear model about trim point

[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys,xs,us);

H = 7.0; %[s] Horizon length

%Controller for system x:
mpc_x = MpcControl_x(sys_x,Ts,H);
mpc_y = MpcControl_y(sys_y,Ts,H);
mpc_z = MpcControl_z(sys_z,Ts,H);
mpc_roll = MpcControl_roll(sys_roll,Ts,H);


%Merge four sub-system controllers into one full-system controller
mpc = rocket.merge_lin_controllers(xs,us,mpc_x,mpc_y,mpc_z,mpc_roll);
% Manipulate mass for simulation
rocket.mass = 2.13;


%5.1 - Mass Offset

x0 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 1; 0; 3]; %As documented in the instructions
ref = [1.2, 0, 3, 0]';
%ref = @(t_, x_) ref_TVC(t_);
Tf = 30;


%Testing setup_estimator function
[T, X_est, U_est, Ref, Z_hat] = rocket.simulate_est_z(x0, Tf, @mpc.get_u, ref, mpc_z, sys_z);
%[T, X_est, U_est, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);
rocket.anim_rate = 10; %increase this to make animation faster
ph2 = rocket.plotvis(T,X_est,U_est,Ref);
ph2.fig.Name = 'Merged lin. MPC in nonlinear simulation with offset'; %set figure title


%General Simulatation

%[T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);

%Visualize
%rocket.anim_rate = 10; %increase this to make animation faster
%ph = rocket.plotvis(T,X,U,Ref);
%ph.fig.Name = 'Merged lin. MPC in nonlinear simulation'; %set figure title

