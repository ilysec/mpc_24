%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Ac, Bc, Bd_c] = generate_system_cont_cc(params)

    inverse_masses = diag([1/params.model.m1 1/params.model.m2 1/params.model.m3]);

    Ac = inverse_masses * [(-params.model.a12 - params.model.a1o)   params.model.a12                                            0;
                            params.model.a12                        (-params.model.a12 - params.model.a23 - params.model.a2o)   params.model.a23;
                            0                                       params.model.a23                                            (-params.model.a23 - params.model.a3o)];

    Bc = inverse_masses * [1 0;
                           0 1;
                           0 0];

    Bd_c = inverse_masses;
    
end