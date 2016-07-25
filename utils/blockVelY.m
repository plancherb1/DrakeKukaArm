%% function to to return block y vel
function [block_vel, dBlock_vel] = blockVelY(x,blockVelOffset)
    block_vel = x(blockVelOffset+2);
    dBlock_vel = zeros(1,size(x,1));
    dBlock_vel(1,blockVelOffset+2) = 1;
end