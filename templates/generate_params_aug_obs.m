%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [params_aug_obs] = generate_params_aug_obs(params)

    params_aug_obs = params;

    % % % Define the augmented system matrices
     params_aug_obs.model.A = [params.model.A, params.model.Bd; zeros(params.model.nd, params.model.nx), eye(params.model.nd)];
     params_aug_obs.model.B = [params.model.B; zeros(params.model.nd, size(params.model.B, 2))];
     params_aug_obs.model.C = [params.model.C, params.model.Cd];
    
    % % Define the augmented system dimensions
     params_aug_obs.model.nx = params.model.nx + params.model.nd;
     params_aug_obs.constraints.StateMatrix = [params.constraints.StateMatrix, zeros(2*params.model.nx, params.model.nd)];
     params_aug_obs.constraints.StateRHS = params.constraints.StateRHS;
end