addpath(fullfile('..', 'src'));

%close all
%clear all
%clc

%% TODO: This file should produce all the plots for the deliverable
Ts = 1/20;
rocket = Rocket(Ts);
H = 5; % Horizon length in seconds
nmpc = NmpcControl(rocket, H);

% MPC reference with default maximum roll = 15 deg
%ref = @(t_, x_) ref_TVC(t_);
ref = [2; 2; 2; 0];
% MPC reference with specified maximum roll = 50 deg
roll_max = deg2rad(50);
%ref = @(t_, x_) ref_TVC(t_, roll_max);

% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = nmpc.get_u(x, ref);
U_opt(:,end+1) = nan;
ph = rocket.plotvis(T_opt, X_opt, U_opt, ref);

Tf = 5;
x0 = [deg2rad([2 -2 0, -2 2 0]), 0 0 0, 0 0 0]';
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
rocket.anim_rate = 1.0;
rocket.vis(T,X,U);