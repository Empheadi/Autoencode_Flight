function y = saturate(x, xmin, xmax)
%SATURATE Clamp signal between lower and upper bounds.
% Inputs:
%   x    : array
%   xmin : scalar/array lower bound
%   xmax : scalar/array upper bound
% Output:
%   y    : saturated array

y = max(min(x, xmax), xmin);
end
