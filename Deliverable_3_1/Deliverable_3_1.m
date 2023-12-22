addpath(fullfile('..', 'src'));

%close all
%clear all
%clc

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);
H = 7; % Horizon length in seconds mpc x = MpcControl x(sys x, Ts, H);
Tf = 7; % seconds of simulation

%x-component
x_x = [0,0,0,10].';
u_x = mpc_x.get_u(x_x);
[u, T_opt, X_opt, U_opt] = mpc_x.get_u(x_x);
U_opt(:,end+1) = NaN;
% Account for linearization point
%X_opt = 0;
%U_opt = 0;
%ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_y, xs, us); % Plot as usual
[T, X_sub, U_sub] = rocket.simulate_f(sys_y, x_x, Tf, @mpc_x.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us);

%y-component


%z-component
z_z = [0;3];
mpc_z = MpcControl_z(sys_z, Ts, H);
u_z = mpc_z.get_u(z_z);
[u, T_opt, X_opt, U_opt] = mpc_z.get_u(z_z);
U_opt(:,end+1) = NaN;
%X_opt = X_opt
U_opt = U_opt + us(3);
%Closed-Loopc
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_z, xs, us);
%Open-Loop
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, z_z, Tf, @mpc_z.get_u, 0);
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_z, xs, us);


%roll-component
roll_roll = [0;deg2rad(40)];
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
u_i = mpc_roll.get_u(roll_roll);
[u, T_opt, X_opt, U_opt] = mpc_roll.get_u(roll_roll);
U_opt(:,end+1) = NaN;
%X_opt = X_opt
%U_opt = U_opt;
%Closed-Loop
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_roll, xs, us);
%Open-Loop
[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, roll_roll, Tf, @mpc_roll.get_u, 0);
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_roll, xs, us);


