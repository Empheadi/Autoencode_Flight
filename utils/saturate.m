function y = saturate(x, lowerBound, upperBound)
%SATURATE Clamp each element of x into [lowerBound, upperBound].
%   y = saturate(x, lowerBound, upperBound)
%
% Inputs:
%   x          [Nx1 double] Value to be saturated.
%   lowerBound [Nx1 double] Lower saturation bound.
%   upperBound [Nx1 double] Upper saturation bound.
%
% Outputs:
%   y          [Nx1 double] Saturated value.

y = min(max(x, lowerBound), upperBound);
end
