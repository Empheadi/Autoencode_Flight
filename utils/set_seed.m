function set_seed(seed)
%SET_SEED Set deterministic RNG seed for reproducible simulations.
%   set_seed(seed)
%
% Inputs:
%   seed [1x1 double] RNG seed value.

rng(seed);
end
