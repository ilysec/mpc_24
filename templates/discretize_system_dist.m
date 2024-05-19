%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Ad, Bd, Bd_d] = discretize_system_dist(Ac, Bc, Bd_c, params)

    A = Ac;
    B = [Bc Bd_c];
    C = zeros(size(Ac));
    D = [zeros(size(Bc)) zeros(size(Bd_c))];

    sys_cont = ss(A, B, C, D);
   
    sys_disc = c2d(sys_cont, params.model.TimeStep, 'zoh');
    
    Ad = sys_disc.A;
    Bd = sys_disc.B(:, 1:(size(Bc,2)));
    Bd_d = sys_disc.B(:, size(Bc,2)+1:(size(Bc,2)+size(Bd_c,2)));

end