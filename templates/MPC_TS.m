%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TS
    properties
        yalmip_optimizer
    end

    methods
        function obj = MPC_TS(Q,R,N,H,h,params)
            A = params.model.A;
            B = params.model.B;
            Hx = params.constraints.StateMatrix;
            hx = params.constraints.StateRHS;
            Hu = params.constraints.InputMatrix;
            hu = params.constraints.InputRHS;
            nx = size(A, 1); 
            nu = size(B, 2); 
            
            X = sdpvar(nx, N+1); 
            U = sdpvar(nu, N); 
            X0 = sdpvar(nx, 1);

            [~, P, ~] = dlqr(A, B, Q, R);

            objective = 0;
            constraints = [];

            for k = 1:N
                objective = objective + X(:,k)' * Q * X(:,k) + U(:,k)' * R * U(:,k);

                constraints = [constraints, ...
                    X(:,k+1) == A * X(:,k) + B * U(:,k), ...
                    Hx * X(:,k) <= hx, ...
                    Hu * U(:,k) <= hu];
            end

            objective = objective + X(:,N+1)' * P * X(:,N+1);

            constraints = [constraints, ...
                H * X(:,N+1) <= h, ...
                X(:,1) == X0, ...
                ];

            opts = sdpsettings('verbose', 1, 'solver', 'quadprog');
            obj.yalmip_optimizer = optimizer(constraints, objective, opts, X0, {U(:,1), objective});
        end

        function [u, ctrl_info] = eval(obj,x)
            %% evaluate control action by solving MPC problem, e.g.
            tic
            [optimizer_out,errorcode] = obj.yalmip_optimizer(x);
            solvetime = toc;
            [u, objective] = optimizer_out{:};

            feasible = true;
            if (errorcode ~= 0)
                feasible = false;
            end

            ctrl_info = struct('ctrl_feas',feasible,'objective',objective,'solvetime',solvetime);
        end
    end
end