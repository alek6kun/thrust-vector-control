addpath(fullfile('..', 'src'));

%close all
%clear all
%clc

%% TODO: This file should produce all the plots for the deliverable

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);
H = 7; % Horizon length in seconds mpc x = MpcControl x(sys x, Ts, H);
Tf = 30; % seconds of simulation

%x-component

%y-component

%z-component
z_z = [0;0];
ref_x = -4;
mpc_z = MpcControl_z(sys_z, Ts, H);
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, z_z, Tf, @mpc_z.get_u, ref_x);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us, ref_x);

% Simulate
%Tf = 30;
%[T, X, U, Ref] = rocket.simulate(z_z, Tf, @mpc_z.get_u, ref_x);
%rocket.anim_rate = 2.0;
%ph2 = rocket.plotvis(T, X, U, Ref);
%ph2.fig.Name = '3.2 z-component'; % Set a figure title

%roll-component
roll_roll = [0;deg2rad(40)];
ref_roll = deg2rad(35);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, roll_roll, Tf, @mpc_roll.get_u, ref_roll);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us, ref_roll);







%Plot terminal invariant sets
%Xf.projection(1:2).plot();
%Xf.projection(2:3).plot();
%Xf.projection(3:4).plot();
