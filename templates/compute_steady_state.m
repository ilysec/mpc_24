%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x_s, u_s] = compute_steady_state(params, d)

    % x_s = params.model.C\((params.model.C_ref\params.exercise.T_ref) - params.model.Cd*d);
    % u_s = params.model.B\((-params.model.A + eye(size(params.model.A))) * x_s - params.model.Bd * d);

    A = params.model.A;
    B = params.model.B;
    Bd = params.model.Bd;
    C = params.model.C;
    Cd = params.model.Cd;
    C_ref = params.model.C_ref;
    T_ref = params.exercise.T_ref;

    % LHS_1 = C_ref * C;
    % RHS_1 = T_ref - C_ref * Cd * d;
    % x_s = linsolve(LHS_1, RHS_1);
    % % x_s = transpose(RHS_1 \ LHS_1); 

    % LHS_2 = B;
    % RHS_2 = x_s - A*x_s - Bd * d;
    % u_s = linsolve(LHS_2, RHS_2);
    % % u_s = transpose(RHS_2 \ LHS_2);

    LEFT = [
        eye(size(A, 1))-A,  -B;
        C_ref*C,            zeros(size(B, 2));
    ];
    RIGHT = [zeros(size(A, 1), 1); T_ref-C_ref*Cd*d];
    result = linsolve(LEFT, RIGHT);
    x_s = result(1:size(A, 1));
    u_s = result((size(A, 1)+1):size(result, 1));
    
end
