%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H, h] = lqr_maxPI(Q, R, params)

    % Extract system matrices and constraints from params
    A = params.model.A;
    B = params.model.B;
    Hu = params.constraints.InputMatrix;
    hu = params.constraints.InputRHS;
    Hx = params.constraints.StateMatrix;
    hx = params.constraints.StateRHS;
    
    % Compute the LQR gain
    [K, ~, ~] = dlqr(A, B, Q, R);
    A_cl = A - B * K;

    stateConstraints = Polyhedron(Hx, hx);
    inputConstraints = Polyhedron(Hu, hu);
    
    % Define the LTI system
    model = LTISystem('A', A, 'B', B);
    model.x.with('setConstraint');
    model.x.setConstraint = stateConstraints;
    model.u.with('setConstraint');
    model.u.setConstraint = inputConstraints;

    % Compute the invariant set
    InvSet = model.invariantSet();

    % Extract the H and h matrices from the polyhedral representation
    H = InvSet.A;
    h = InvSet.b;

end

