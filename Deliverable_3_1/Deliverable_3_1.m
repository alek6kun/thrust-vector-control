addpath(fullfile('..', 'src'));

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

%x-component
mpc_x = MpcControl_x(sys_x, Ts, H);
x_x = [0,0,0,3].';


%Open-Loop
[u_x, T_opt, X_opt, U_opt] = mpc_x.get_u(x_x);
U_opt(:,end+1) = NaN;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us);
saveas(gcf,'Open-loop x.png')
%Closed-Loop

[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x_x, Tf, @mpc_x.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);
saveas(gcf,'Closed-loop x.png')
%y-component
mpc_y = MpcControl_y(sys_y, Ts, H);
x_y = [0,0,0,3].';

% Account for linearization point

%Open-Loop
[u_y, T_opt, X_opt, U_opt] = mpc_y.get_u(x_y);
U_opt(:,end+1) = NaN;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_y, xs, us);
saveas(gcf,'Open-loop y.png')
%Closed-Loop
[T, X_sub, U_sub] = rocket.simulate_f(sys_y, x_x, Tf, @mpc_y.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us);
saveas(gcf,'Closed-loop y.png')
%z-component
z_z = [0;3];
mpc_z = MpcControl_z(sys_z, Ts, H);
[u_z, T_opt, X_opt, U_opt] = mpc_z.get_u(z_z);
U_opt(:,end+1) = NaN;

U_opt = U_opt + us(3);
%Open-Loop
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_z, xs, us);

%Closed-Loop
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, z_z, Tf, @mpc_z.get_u, 0);
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_z, xs, us);




%roll-component
roll_roll = [0;deg2rad(30)];
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
[u_i, T_opt, X_opt, U_opt] = mpc_roll.get_u(roll_roll);
U_opt(:,end+1) = NaN;
%Open-Loop
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_roll, xs, us);

%Closed-Loop
[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, roll_roll, Tf, @mpc_roll.get_u, 0);
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_roll, xs, us);


