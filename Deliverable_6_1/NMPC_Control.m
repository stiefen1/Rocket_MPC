function opti_eval = NMPC_Control(rocket, H)

import casadi.*
opti = casadi.Opti(); % Optimization problem

N = ceil(H/rocket.Ts); % MPC horizon
nx = 12; % Number of states
nu = 4;  % Number of inputs

% Decision variables (symbolic)
X_sym = opti.variable(nx, N); % state trajectory
U_sym = opti.variable(nu, N-1);   % control trajectory)

% Parameters (symbolic)
x0_sym  = opti.parameter(nx, 1);  % initial state
ref_sym = opti.parameter(4, 1);   % target position

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ---- slack variables --------
% epsilon_gamma = opti.variable(1,N); % slack variable for gamma constraint

% ---- boundary conditions --------
opti.subject_to(X_sym(:,1)==x0_sym);

% ---- state constraints -----------
opti.subject_to(-deg2rad(85) <= X_sym(5,:) <= deg2rad(85)); % |Beta| <= 85 deg

% ---- input constraints -----------
opti.subject_to(-deg2rad(15) <= U_sym(1,:) <= deg2rad(15));
opti.subject_to(-deg2rad(15) <= U_sym(2,:) <= deg2rad(15));
opti.subject_to(50 <= U_sym(3,:) <= 80);
opti.subject_to(-20 <= U_sym(4,:) <= 20);

Pavg_zero = 56.6666665401736;

% ---- objective ---------
opti.minimize(...
  1000   *(X_sym(10,:)-ref_sym(1,1))*(X_sym(10,:)-ref_sym(1,1))'  + ... % X
  1000   *(X_sym(11,:)-ref_sym(2,1))*(X_sym(11,:)-ref_sym(2,1))'  + ... % Y
  1000   *(X_sym(12,:)-ref_sym(3,1))*(X_sym(12,:)-ref_sym(3,1))'  + ... % Z
   400   *(X_sym(6 ,:)-ref_sym(4,1))*(X_sym(6 ,:)-ref_sym(4,1))'  + ... % Roll (gamma)
   400   *X_sym(4 ,:)*X_sym(4 ,:)'  + ... % Alpha
   400   *X_sym(5 ,:)*X_sym(5,:)'  + ... % Beta
  10     *X_sym(1,:)*X_sym(1,:)' + ... 
  10     *X_sym(2,:)*X_sym(2,:)' + ...
  10     *X_sym(3,:)*X_sym(3,:)' + ...
  100*U_sym(1,:)*U_sym(1,:)' + ... % Minimize d1
  100*U_sym(2,:)*U_sym(2,:)' + ... % Minimize d2
  1*(U_sym(3,:)-Pavg_zero)*(U_sym(3,:)-Pavg_zero)' + ... % Minimize Pavg
  1*U_sym(4,:)*U_sym(4,:)' + ... % Minimize Pdiff
  0); % soft constraints

f = @(x, u) rocket.f(x, u);
for k=1:N-1
  opti.subject_to(X_sym(:,k+1) == Euler(X_sym(:,k),U_sym(:,k),rocket.Ts,f));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ---- Setup solver ------
ops = struct('ipopt', struct('print_level', 0, 'tol', 1e-3), 'print_time', false);
opti.solver('ipopt', ops);

% Create function to solve and evaluate opti
opti_eval = @(x0_, ref_) solve(x0_, ref_, opti, x0_sym, ref_sym, U_sym);
end

function u = solve(x0, ref, opti, x0_sym, ref_sym, U_sym)

% ---- Set the initial state and reference ----
opti.set_value(x0_sym, x0);
opti.set_value(ref_sym, ref);

% ---- Solve the optimization problem ----
sol = opti.solve();
assert(sol.stats.success == 1, 'Error computing optimal input');

u = opti.value(U_sym(:,1));

% Use the current solution to speed up the next optimization
opti.set_initial(sol.value_variables());
opti.set_initial(opti.lam_g, sol.value(opti.lam_g));
end
