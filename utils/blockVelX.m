%% function to to return block x vel
function [block_vel, dBlock_vel] = blockVelX(x,blockVelOffset)
    block_vel = x(blockVelOffset+1);
    dBlock_vel = zeros(1,size(x,1));
    dBlock_vel(1,blockVelOffset+1) = 1;
end