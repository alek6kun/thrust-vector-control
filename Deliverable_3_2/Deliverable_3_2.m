addpath(fullfile('..', 'src'));
addpath(fullfile('..', 'MpcControl/'));
close all
clear
clc

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);
H = 7; % Horizon length in seconds mpc x = MpcControl x(sys x, Ts, H);
Tf = 10; % seconds of simulation

%x-component
x_x = [0;0;0;0];
ref_x = -4;
mpc_x = MpcControl_x(sys_x, Ts, H);
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x_x, Tf, @mpc_x.get_u, ref_x);
%ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us, ref_x);
%y-component
x_y = [0;0;0;0];
ref_y = -4;
mpc_y = MpcControl_x(sys_y, Ts, H);
[T, X_sub, U_sub] = rocket.simulate_f(sys_y, x_y, Tf, @mpc_y.get_u, ref_y);
%ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us, ref_y);

%z-component
z_z = [0;0];
ref_z = -4;
mpc_z = MpcControl_z(sys_z, Ts, H);

%Open-Loop
[u, T_opt, X_opt, U_opt] = mpc_z.get_u(z_z);
U_opt(:,end+1) = NaN;
U_opt = U_opt + us(3);
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_z, xs, us, ref_z); 

%Closed-Loop
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, z_z, Tf, @mpc_z.get_u, ref_z);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us, ref_z);




%roll-component
roll_roll = [0;0];
ref_roll = deg2rad(35);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, roll_roll, Tf, @mpc_roll.get_u, ref_roll);
%Closed-Loop
%ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us, ref_z);

%Open-Loop
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, z_z, Tf, @mpc_z.get_u, ref_z);
%ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us, ref_z);

