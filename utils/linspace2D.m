%% compute 2D linspace
function [out] = linspace2D(x0,xf,N)
    if nargin < 3
        N = 100;
    end
    xDim = size(x0,1);
    out = zeros(xDim,N);
    for row = 1:xDim
        out(row,:) = linspace(x0(row),xf(row),N);
    end
end