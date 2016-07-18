%% function returns difference in block and hand velocity
function [ret,dRet] = handBlockVelError(p,x,blockVelOffset,magnitude_flag)
    [hand_vel, dHand_vel] = handVel(p,x);
    [block_vel, dBlock_vel] = blockVel(x,blockVelOffset);
    delta_vel = hand_vel - block_vel;
    dDelta_vel = dHand_vel - dBlock_vel;
    if magnitude_flag
        delta_vel_squared = delta_vel'*eye(size(delta_vel,1))*delta_vel;
        dDelta_vel_squared = 1/2*delta_vel'*dDelta_vel;
        vel_err = sqrt(delta_vel_squared);
        dVel_err = 1/2*delta_vel_square * dDelta_vel_squared;
        ret = vel_err;
        dRet = dVel_err;
    else
        ret = delta_vel;
        dRet = dDelta_vel;
    end
end