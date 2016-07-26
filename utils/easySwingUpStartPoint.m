%% Compute the easySwingUpStart point for a Kuka arm
function [xf,uf] = easySwingUpStartPoint(p,v,difficultyLevel)
    if nargin < 3
        difficultyLevel = 1;
    end
    xf = zeros(p.num_positions + p.num_velocities,1);
    switch difficultyLevel
        case 1
            xf(2) = 0.25;
        case 2
            xf(2) = 0.35;
        case 3
            xf(2) = 0.5;
        case 4
            xf(2) = 0.75;
        case 5
            xf(2) = 1;
        otherwise
            xf(2) = pi/2;
    end
    uf = zeros(size(p.umin,1),1);
end