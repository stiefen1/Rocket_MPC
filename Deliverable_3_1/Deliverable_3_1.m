%% Initialisation
close all; clear; clc;
yalmip('clear');

addpath(fullfile('..', 'src'));

Ts = 1/20;
rocket = Rocket(Ts);
Tf = 5.0;

[xs,us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

H = 4; % Horizon length in seconds



%% Control X
mpc_x = MPC_Control_x(sys_x, Ts, H);

% % Get control input
x0 = [deg2rad([0 0]) 0 1]';

% % Visualization
[T, X_sub, U_sub] = rocket.simulate(sys_x, x0, Tf, @mpc_x.get_u, 0);
rocket.anim_rate = 1.0;
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);

%% Control Y
mpc_y = MPC_Control_y(sys_y, Ts, H);

% % Get control input
x0 = [deg2rad([0 0]) 0 1]';
 
% % Visualization
[T, X_sub, U_sub] = rocket.simulate(sys_y, x0, Tf, @mpc_y.get_u, 0);
rocket.anim_rate = 1.0;
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us);

%% Control Z
mpc_z = MPC_Control_z(sys_z, Ts, H);

% % Get control input
x0 = [0 1]';

% % Visualization
[T, X_sub, U_sub] = rocket.simulate(sys_z, x0, Tf, @mpc_z.get_u, 0);
rocket.anim_rate = 1.0;
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z,xs, us);

%% Control roll
mpc_roll = MPC_Control_roll(sys_roll, Ts, H);

% % Get control input
x0 = [0 deg2rad(30)]';

% % Visualization
[T, X_sub, U_sub] = rocket.simulate(sys_roll, x0, Tf, @mpc_roll.get_u, 0);
rocket.anim_rate = 1.0;
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us);
