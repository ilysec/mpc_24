%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x_s, u_s] = compute_steady_state(params, d)

    A = params.model.A;
    B = params.model.B;
    Bd = params.model.Bd;
    C = params.model.C;
    Cd = params.model.Cd;
    C_ref = params.model.C_ref;
    T_ref = params.exercise.T_ref;

    LEFT = [
        eye(size(A, 1))-A,  -B;
        C_ref*C,            zeros(size(B, 2));
    ];
    RIGHT = [Bd * d;  T_ref - C_ref * Cd * d];
    result = pinv(LEFT) * RIGHT;
    x_s = result(1:size(A, 1));
    u_s = result((size(A, 1)+1):size(result, 1));
    
end
