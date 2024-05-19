%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function params_delta = generate_params_delta_cc(params)
    
params_delta = params;

params_delta.model = rmfield(params_delta.model, 'C');
params_delta.model = rmfield(params_delta.model, 'Cd');
params_delta.model = rmfield(params_delta.model, 'C_ref');
params_delta.model = rmfield(params_delta.model, 'D');
params_delta.model = rmfield(params_delta.model, 'Bd');

% CONTINUE BELOW THIS LINE

d = [params.model.a1o; params.model.a2o; params.model.a3o] * params.exercise.To + params.exercise.RadiationA;

[params_delta.exercise.x_s , params_delta.exercise.u_s] = compute_steady_state(params, d);

params_delta.exercise.InitialConditionA = params_delta.exercise.InitialConditionA - params_delta.exercise.x_s;
params_delta.exercise.InitialConditionB = params_delta.exercise.InitialConditionB - params_delta.exercise.x_s; 
params_delta.exercise.InitialConditionC = params_delta.exercise.InitialConditionC - params_delta.exercise.x_s;

params_delta.constraints.InputRHS = params_delta.constraints.InputRHS - params.constraints.InputMatrix * params_delta.exercise.u_s;

params_delta.constraints.StateRHS = params_delta.constraints.StateRHS - params.constraints.StateMatrix * params_delta.exercise.x_s;


end