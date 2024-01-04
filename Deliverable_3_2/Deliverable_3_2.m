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
H = 10; % Horizon length in seconds mpc x = MpcControl x(sys x, Ts, H);
Tf = 10; % seconds of simulation

%%%%%%%%%%%%%%%%%%%%%%% x-component %%%%%%%%%%%%%%%%%%%%%%%

x_x = [0;0;0;0];
ref_x = -4;
mpc_x = MpcControl_x(sys_x, Ts, H);
%OL
[u_x, T_opt_x, X_opt_x, U_opt_x] = mpc_x.get_u(x_x, ref_x);
U_opt_x(:,end+1) = NaN;
ph = rocket.plotvis_sub(T_opt_x, X_opt_x, U_opt_x, sys_x, xs, us);
saveas(gcf,'3.2 OL x.png')
%CL
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x_x, Tf, @mpc_x.get_u, ref_x);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us, ref_x);
saveas(gcf,'3.2 CL x.png')


%%%%%%%%%%%%%%%%%%%%%%% y-component %%%%%%%%%%%%%%%%%%%%%%%


x_y = [0;0;0;0];
ref_y = -4;
mpc_y = MpcControl_x(sys_y, Ts, H);
%OL
[u_y, T_opt_y, X_opt_y, U_opt_y] = mpc_y.get_u(x_y, ref_y);
U_opt_y(:,end+1) = NaN;
ph = rocket.plotvis_sub(T_opt_y, X_opt_y, U_opt_y, sys_y, xs, us);
saveas(gcf,'3.2 OL y.png')
%CL
[T, X_sub, U_sub] = rocket.simulate_f(sys_y, x_y, Tf, @mpc_y.get_u, ref_y);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us, ref_y);
saveas(gcf,'3.2 CL y.png')


%%%%%%%%%%%%%%%%%%%%%%% z-component %%%%%%%%%%%%%%%%%%%%%%%

z_z = [0;0];
ref_z = -4;
mpc_z = MpcControl_z(sys_z, Ts, H);

%OL
[u_z, T_opt_z, X_opt_z, U_opt_z] = mpc_z.get_u(z_z, ref_z);
U_opt_z(:,end+1) = NaN;
U_opt_z = U_opt_z + us(3);
ph = rocket.plotvis_sub(T_opt_z, X_opt_z, U_opt_z, sys_z, xs, us);
saveas(gcf,'3.2 OL z.png')
%CL
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, z_z, Tf, @mpc_z.get_u, ref_z);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us, ref_z);
saveas(gcf,'3.2 CL z.png')

%%%%%%%%%%%%%%%%%%%%%%% roll-component %%%%%%%%%%%%%%%%%%%%%%%

roll_roll = [0;0];
ref_roll = deg2rad(35);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

%OL
[u_roll, T_opt_roll, X_opt_roll, U_opt_roll] = mpc_roll.get_u(roll_roll, ref_roll);
U_opt_roll(:,end+1) = NaN;
U_opt_z = U_opt_z + us(3);
ph = rocket.plotvis_sub(T_opt_roll, X_opt_roll, U_opt_roll, sys_roll, xs, us);
saveas(gcf,'3.2 OL roll.png')
%CL
[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, roll_roll, Tf, @mpc_roll.get_u, ref_roll);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us, ref_roll);
saveas(gcf,'3.2 CL roll.png')




