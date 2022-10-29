%% Init
close all; clear; clc;
yalmip('clear');

addpath(fullfile('..', 'src'));
addpath('integrators');

%% Create rocket

Ts = 1/10;
rocket = Rocket(Ts);

%% Create controller
H = 1;
nmpc = NMPC_Control(rocket, H);

%% Simulate in closed-loop using NMPC
Tf = 30;
ref = @(t , x ) rocket.MPC_ref(t , Tf);
x0 = zeros(12,1);

tic
[T, X, U, Ref] = rocket.simulate_f(x0, Tf, nmpc, ref);
toc
rocket.anim_rate = 4; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Nonlin. sim...........'; % Set a figure title

