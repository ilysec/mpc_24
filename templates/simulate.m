%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x,u,ctrl_info] = simulate(x0, ctrl, params)

        x = zeros(params.model.nx, params.exercise.SimHorizon);
        x(:,1) = x0;
        u = zeros(params.model.nu, params.exercise.SimHorizon);
        ctrl_info.ctrl_feas = true;
        
        for i = 1:params.exercise.SimHorizon  
            [u(:, i), info] = ctrl.eval(x(:, i));
            ctrl_info(i).ctrl_feas = info.ctrl_feas;
            x(:, i+1) = params.model.A * x(:, i) + params.model.B * u(:, i);
        end
end