addpath(fullfile('..', 'src'));
close all
clear
clc

%% Initialisations
Ts = 1/40; % Higher sampling rate for this part!

% Define NMPC
rocket = Rocket(Ts);
H = 3; % Horizon length in seconds
nmpc = NmpcControl(rocket, H);


x0 = zeros(12, 1);
ref = [0.5, 0, 1, deg2rad(65)]';
Tf = 2.5;
rocket.mass = 1.75;
rocket.anim_rate = 2.0;

%% Delay which causes drop in performance

rocket.delay = 3;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

ph1 = rocket.plotvis(T, X, U, Ref);
ph1.fig.Name = '75ms delay causing drop in performance';

%% Delay which causes unstability

rocket.delay = 4;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

ph2 = rocket.plotvis(T, X, U, Ref);
ph2.fig.Name = '100ms delay causing unstability';

%% Partially compensated delay

nmpc = NmpcControl(rocket, H, 7);
rocket.delay = 8;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

ph3 = rocket.plotvis(T, X, U, Ref);
ph3.fig.Name = 'Partial compensation (175ms) of 200ms delay';

%% Fully compensated delay

nmpc = NmpcControl(rocket, H, 8);
rocket.delay = 8;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

ph3 = rocket.plotvis(T, X, U, Ref);
ph3.fig.Name = 'Full delay compensation of 200ms delay';

