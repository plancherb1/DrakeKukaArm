%% function to retrun block height
function [z] = ballHeight(x,blockPosOffset)
    z = x(blockPosOffset+3) - 0.2; %0.2 is radius
end