%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [T1_max, T2_min, T2_max, P1_min, P1_max, P2_min, P2_max, input_cost, cstr_viol] = traj_constraints_cc(X, U, params)




    T1_max = max(X(1,:));
    T2_min = min(X(2,:));
    T2_max = max(X(2,:));
    P1_min = min(U(1,:));
    P1_max = max(U(1,:));
    P2_min = min(U(2,:));
    P2_max = max(U(2,:));
    
    T_t1_max = params.constraints.T1Max;
    T_t2_min = params.constraints.T2Min;
    T_t2_max = params.constraints.T2Max;
    pt1_min = params.constraints.P1Min;
    pt1_max = params.constraints.P1Max;
    pt2_min = params.constraints.P2Min;
    pt2_max = params.constraints.P2Max;
    
    input_cost = 0;
    for i = 1:size(U,2)
        input_cost = input_cost + U(:,i)'*U(:,i);
    end

    cstr_viol = (T1_max > T_t1_max || T2_min < T_t2_min || T2_max > T_t2_max || ...
             P1_max > pt1_max || P2_min < pt2_min || P1_min < pt1_min || ...
             P2_max > pt2_max);
end

