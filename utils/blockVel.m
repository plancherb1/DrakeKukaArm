%% function to to return block vel and its norm
function [ret, dRet] = blockVel(x,blockVelOffset,magnitude_flag)
    block_vel = x(blockVelOffset+1:blockVelOffset+3);
    dBlock_vel = zeros(1,size(x,1));
    dBlock_vel(1,blockVelOffset+1) = 1;
    dBlock_vel(2,blockVelOffset+2) = 1;
    dBlock_vel(3,blockVelOffset+3) = 1;
    if magnitude_flag
        velSq = block_vel'*eye(size(block_vel,1))*block_vel;
        dVelSq = 2*block_vel'*dBlock_vel;
        vel = sqrt(velSq);
        dVel = 1/2*velSq * dVelSq;
        ret = vel;
        dRet = dVel;
    else
        ret = block_vel;
        dRet = dBlock_vel;
    end
end