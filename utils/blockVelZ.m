%% function to to return block z vel
function [block_vel, dBlock_vel] = blockVelZ(x,blockVelOffset)
    block_vel = x(blockVelOffset+3);
    dBlock_vel = zeros(1,size(x,1));
    dBlock_vel(1,blockVelOffset+3) = 1;
end