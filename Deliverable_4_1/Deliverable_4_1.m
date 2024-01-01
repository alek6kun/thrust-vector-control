%% To Do 4.1
clc
clear
close all
addpath(fullfile('..', 'src'));
addpath(fullfile('..', 'MpcControl/'));

Ts = 1/20;
rocket = Rocket(Ts);

[xs,us] = rocket.trim(); %Compute steady-state for which 0 = f(xs,us)
sys = rocket.linearize(xs,us); %Linearize the nonlinear model about trim point

[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys,xs,us);

H = 2.0; %[s] Horizon length

%Controller for system x:
mpc_x = MpcControl_x(sys_x,Ts,H);
mpc_y = MpcControl_y(sys_y,Ts,H);
mpc_z = MpcControl_z(sys_z,Ts,H);
mpc_roll = MpcControl_roll(sys_roll,Ts,H);

%This comes from the data To Do 4.1 
%Merge fous sub-system controllers into one full-system controller
mpc = rocket.merge_lin_controllers(xs,us,mpc_x,mpc_y,mpc_z,mpc_roll);


%Evaluate once and plot optimal open-loop trajectory,
%pad last input to get consistent size with time and state
x0 = zeros(12,1);
ref4 = [2 2 2 deg2rad(40)]';
[u, T_opt, X_opt, U_opt] = mpc.get_u(x0,ref4);
U_opt(:,end+1) = nan;
ph = rocket.plotvis(T_opt, X_opt, U_opt, ref4); %plot as usual


ref = @(t_, x_) ref_TVC(t_);


%Simulate
Tf = 30;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);

%Visualize
rocket.anim_rate = 10; %increase this to make animation faster
ph = rocket.plotvis(T,X,U,Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation'; %set figure title

