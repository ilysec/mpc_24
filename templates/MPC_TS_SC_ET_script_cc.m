%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ADD STUFF HERE

params = generate_params_cc();
A = params.model.A;
B = params.model.B;
nx = params.model.nx;
nu = params.model.nu;

Q = diag(params.exercise.QdiagOpt);
R = diag(params.exercise.RdiagOpt);
N = 30;

x0_c = params.exercise.InitialConditionC;
x0_b = params.exercise.InitialConditionB;

Hx = params.constraints.StateMatrix;
hx = params.constraints.StateRHS;
Hu = params.constraints.InputMatrix;
hu = params.constraints.InputRHS;
H = Hx;
h = hx;

S = 1 * eye(size(hx, 1));
v = 50;
S_t = 1 * eye(size(H, 1));
v_t = 50;

mpc_test = MPC_TS(Q, R, N, H, h, params);
mpc_controller = MPC_TS_SC_ET(Q, R, N, H, h, S, v, S_t, v_t, params);

x_sc = x0_c;
[u, ctrl_info] = mpc_controller.eval(x_sc);

x_ts = x0_b;
[u_ts, ctrl_info_ts] = mpc_test.eval(x_ts);

disp('Control Input HARD:');
disp(u_ts);
disp('Control Info HARD:');
disp(ctrl_info_ts);

disp('Control Input:');
disp(u);
disp('Control Info:');
disp(ctrl_info);

%% Save
current_folder = fileparts(which(mfilename));
save(fullfile(current_folder, "MPC_TS_SC_ET_script_cc.mat"), 'v', 'S', 'v_t', 'S_t');

