close all; clear; clc;
yalmip('clear');

addpath(fullfile('..', 'src'));

Ts = 1/20;
rocket = Rocket(Ts);

[xs,us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

H = 1; % Horizon length in seconds

mpc_x = MPC_Control_x(sys_x, Ts, H);
mpc_y = MPC_Control_y(sys_y, Ts, H);
mpc_z = MPC_Control_z(sys_z, Ts, H);
mpc_roll = MPC_Control_roll(sys_roll, Ts, H);

% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);
% Setup reference function
Tf = 30;
ref = @(t , x ) rocket.MPC_ref(t , Tf);
x0 = zeros(12,1);
[T, X, U, Ref] = rocket.simulate_f(x0, Tf, mpc, ref);
% Plot pose
anim_rate = 4; %is about right for printing in the report
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Nonlin. sim...........'; % Set a figure title
